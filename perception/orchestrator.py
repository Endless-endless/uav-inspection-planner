"""Minimal in-process orchestrator for the 8002 -> 8003 perception chain."""

from __future__ import annotations

import asyncio
import time
from pathlib import Path
from typing import Any, Awaitable, Callable, Mapping

from config.perception import (
    VIDEO_POLL_DEADLINE_SECONDS,
    VIDEO_POLL_INTERVAL_SECONDS,
)
from perception.clients.defect_detection import DefectDetectionClient
from perception.clients.video_recognition import VideoRecognitionClient
from perception.mission_identity import MissionSnapshot
from perception.models import FrameResult, PerceptionWorkflowResult, WorkflowError


EventCallback = Callable[[str, Mapping[str, Any]], None]
SleepFunction = Callable[[float], Awaitable[None]]
ClockFunction = Callable[[], float]


class PerceptionOrchestrator:
    """Run one video workflow without persistence, routing, or background workers."""

    def __init__(
        self,
        *,
        video_client: VideoRecognitionClient,
        defect_client: DefectDetectionClient,
        poll_interval_seconds: float = VIDEO_POLL_INTERVAL_SECONDS,
        poll_deadline_seconds: float = VIDEO_POLL_DEADLINE_SECONDS,
        sleep: SleepFunction = asyncio.sleep,
        clock: ClockFunction = time.monotonic,
        event_callback: EventCallback | None = None,
    ) -> None:
        if poll_interval_seconds < 0:
            raise ValueError("poll_interval_seconds must be non-negative")
        if poll_deadline_seconds <= 0:
            raise ValueError("poll_deadline_seconds must be greater than zero")
        self._video_client = video_client
        self._defect_client = defect_client
        self._poll_interval = poll_interval_seconds
        self._poll_deadline = poll_deadline_seconds
        self._sleep = sleep
        self._clock = clock
        self._event_callback = event_callback

    async def run(
        self,
        *,
        mission_snapshot: MissionSnapshot,
        video_path: str | Path,
        inspection_point_id: str,
        video_id: str | None = None,
        runtime_mission_id: str | None = None,
    ) -> PerceptionWorkflowResult:
        """Run one complete video-to-defect workflow serially."""
        mission_snapshot.require_inspection_point(inspection_point_id)
        mission_id = runtime_mission_id or mission_snapshot.mission_id
        if not isinstance(mission_id, str) or not mission_id:
            raise ValueError("runtime mission_id must be a non-empty string")

        video_file = Path(video_path)
        video_job_id: str | None = None
        resolved_video_id = video_id
        try:
            video_bytes = video_file.read_bytes()
            create_response = await self._video_client.create_job(
                video_bytes=video_bytes,
                filename=video_file.name,
                mission_id=mission_id,
                inspection_point_id=inspection_point_id,
                video_id=video_id,
            )
            video_job_id = self._required_string(create_response, "job_id")
            response_video_id = create_response.get("video_id")
            if isinstance(response_video_id, str) and response_video_id:
                resolved_video_id = response_video_id
            self._emit(
                "video_job_created",
                {"job_id": video_job_id, "status": create_response.get("status")},
            )
            completed_job = await self._poll_video_job(video_job_id)
        except Exception as exc:
            error = WorkflowError(
                code=self._workflow_error_code(exc), message=str(exc)
            )
            return self._result(
                status="failed",
                mission_id=mission_id,
                mission_sha256=mission_snapshot.mission_sha256,
                inspection_point_id=inspection_point_id,
                video_job_id=video_job_id,
                video_id=resolved_video_id,
                errors=(error,),
            )

        if completed_job.get("status") == "failed":
            upstream_error = completed_job.get("error")
            error = WorkflowError(
                code="video_job_failed",
                message=self._error_message(upstream_error, "video job failed"),
            )
            return self._result(
                status="failed",
                mission_id=mission_id,
                mission_sha256=mission_snapshot.mission_sha256,
                inspection_point_id=inspection_point_id,
                video_job_id=video_job_id,
                video_id=resolved_video_id,
                errors=(error,),
            )

        target_frames = completed_job.get("target_frames")
        if not isinstance(target_frames, list):
            error = WorkflowError(
                code="invalid_video_response",
                message="completed video job target_frames must be an array",
            )
            return self._result(
                status="failed",
                mission_id=mission_id,
                mission_sha256=mission_snapshot.mission_sha256,
                inspection_point_id=inspection_point_id,
                video_job_id=video_job_id,
                video_id=resolved_video_id,
                errors=(error,),
            )

        completed_video_id = completed_job.get("video_id")
        if isinstance(completed_video_id, str) and completed_video_id:
            resolved_video_id = completed_video_id
        self._emit("target_frames", {"count": len(target_frames)})

        frame_results: list[FrameResult] = []
        errors: list[WorkflowError] = []
        for index, frame in enumerate(target_frames):
            validation_error = self._validate_frame(
                frame,
                mission_id=mission_id,
                inspection_point_id=inspection_point_id,
                index=index,
            )
            if validation_error is not None:
                errors.append(validation_error)
                frame_result = FrameResult(
                    frame_id=self._optional_string(frame, "frame_id"),
                    timestamp_ms=self._optional_timestamp(frame),
                    source_image_url=self._optional_string(frame, "image_url"),
                    status=(
                        "identity_mismatch"
                        if validation_error.code == "identity_mismatch"
                        else "invalid_frame"
                    ),
                    error=validation_error,
                )
                frame_results.append(frame_result)
                self._emit("frame_failed", frame_result.to_dict())
                continue

            assert isinstance(frame, Mapping)
            frame_id = frame["frame_id"]
            timestamp_ms = frame["timestamp_ms"]
            image_url = frame["image_url"]
            try:
                media = await self._video_client.download_media(image_url)
            except Exception as exc:
                error = WorkflowError(
                    code="frame_download_failed",
                    message=str(exc),
                    frame_id=frame_id,
                )
                errors.append(error)
                frame_result = FrameResult(
                    frame_id=frame_id,
                    timestamp_ms=timestamp_ms,
                    source_image_url=image_url,
                    status="download_failed",
                    error=error,
                )
                frame_results.append(frame_result)
                self._emit("frame_failed", frame_result.to_dict())
                continue

            try:
                defect_result = await self._defect_client.detect_image(
                    image_bytes=media.content,
                    filename=media.filename,
                    content_type=media.content_type,
                    mission_id=mission_id,
                    inspection_point_id=inspection_point_id,
                    frame_id=frame_id,
                    timestamp_ms=timestamp_ms,
                )
            except Exception as exc:
                error = WorkflowError(
                    code="defect_detection_failed",
                    message=str(exc),
                    frame_id=frame_id,
                )
                errors.append(error)
                frame_result = FrameResult(
                    frame_id=frame_id,
                    timestamp_ms=timestamp_ms,
                    source_image_url=image_url,
                    status="defect_failed",
                    error=error,
                )
                frame_results.append(frame_result)
                self._emit("frame_failed", frame_result.to_dict())
                continue

            frame_result = FrameResult(
                frame_id=frame_id,
                timestamp_ms=timestamp_ms,
                source_image_url=image_url,
                status="completed",
                defect_detection=defect_result,
            )
            frame_results.append(frame_result)
            self._emit("frame_completed", frame_result.to_dict())

        successful_frames = sum(
            frame.status == "completed" for frame in frame_results
        )
        if not errors:
            final_status = "completed"
        elif successful_frames:
            final_status = "completed_with_errors"
        else:
            final_status = "failed"
            errors.append(
                WorkflowError(
                    code="all_target_frames_failed",
                    message="no target frame completed defect detection",
                )
            )
        return self._result(
            status=final_status,
            mission_id=mission_id,
            mission_sha256=mission_snapshot.mission_sha256,
            inspection_point_id=inspection_point_id,
            video_job_id=video_job_id,
            video_id=resolved_video_id,
            target_frame_count=len(target_frames),
            frames=tuple(frame_results),
            errors=tuple(errors),
        )

    async def _poll_video_job(self, video_job_id: str) -> dict[str, Any]:
        started_at = self._clock()
        last_status: str | None = None
        while True:
            if self._clock() - started_at >= self._poll_deadline:
                raise TimeoutError(
                    f"video job {video_job_id} exceeded polling deadline "
                    f"of {self._poll_deadline:g} seconds"
                )
            job = await self._video_client.get_job(video_job_id)
            status = job.get("status")
            if status not in {"queued", "running", "completed", "failed"}:
                raise ValueError(f"video job returned unsupported status: {status!r}")
            if status != last_status:
                self._emit("video_status", {"job_id": video_job_id, "status": status})
                last_status = status
            if status in {"completed", "failed"}:
                return job
            await self._sleep(self._poll_interval)

    @staticmethod
    def _validate_frame(
        frame: Any,
        *,
        mission_id: str,
        inspection_point_id: str,
        index: int,
    ) -> WorkflowError | None:
        if not isinstance(frame, Mapping):
            return WorkflowError(
                code="invalid_frame", message=f"target_frames[{index}] must be an object"
            )
        if frame.get("mission_id") != mission_id:
            return WorkflowError(
                code="identity_mismatch",
                message=(
                    f"target_frames[{index}].mission_id does not match "
                    "the runtime mission_id"
                ),
                frame_id=PerceptionOrchestrator._optional_string(frame, "frame_id"),
            )
        if frame.get("inspection_point_id") != inspection_point_id:
            return WorkflowError(
                code="identity_mismatch",
                message=(
                    f"target_frames[{index}].inspection_point_id does not match "
                    "the authoritative inspection point"
                ),
                frame_id=PerceptionOrchestrator._optional_string(frame, "frame_id"),
            )
        frame_id = frame.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id:
            return WorkflowError(
                code="invalid_frame",
                message=f"target_frames[{index}].frame_id must be a non-empty string",
            )
        timestamp_ms = frame.get("timestamp_ms")
        if (
            isinstance(timestamp_ms, bool)
            or not isinstance(timestamp_ms, int)
            or timestamp_ms < 0
        ):
            return WorkflowError(
                code="invalid_frame",
                message=(
                    f"target_frames[{index}].timestamp_ms must be a non-negative integer"
                ),
                frame_id=frame_id,
            )
        image_url = frame.get("image_url")
        if not isinstance(image_url, str) or not image_url:
            return WorkflowError(
                code="invalid_frame",
                message=f"target_frames[{index}].image_url must be a non-empty string",
                frame_id=frame_id,
            )
        return None

    def _emit(self, event: str, payload: Mapping[str, Any]) -> None:
        if self._event_callback is not None:
            self._event_callback(event, payload)

    @staticmethod
    def _required_string(payload: Mapping[str, Any], field: str) -> str:
        value = payload.get(field)
        if not isinstance(value, str) or not value:
            raise ValueError(f"video response {field} must be a non-empty string")
        return value

    @staticmethod
    def _optional_string(payload: Any, field: str) -> str | None:
        if isinstance(payload, Mapping):
            value = payload.get(field)
            if isinstance(value, str):
                return value
        return None

    @staticmethod
    def _optional_timestamp(payload: Any) -> int | None:
        if isinstance(payload, Mapping):
            value = payload.get("timestamp_ms")
            if isinstance(value, int) and not isinstance(value, bool):
                return value
        return None

    @staticmethod
    def _workflow_error_code(exc: Exception) -> str:
        if isinstance(exc, TimeoutError):
            return "video_poll_deadline_exceeded"
        if isinstance(exc, OSError):
            return "video_file_error"
        return "video_workflow_failed"

    @staticmethod
    def _error_message(value: Any, fallback: str) -> str:
        if isinstance(value, Mapping):
            message = value.get("message")
            if isinstance(message, str) and message:
                return message
        if isinstance(value, str) and value:
            return value
        return fallback

    @staticmethod
    def _result(**kwargs: Any) -> PerceptionWorkflowResult:
        return PerceptionWorkflowResult(**kwargs)


__all__ = ["EventCallback", "PerceptionOrchestrator"]
