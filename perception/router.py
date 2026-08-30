"""Minimal FastAPI surface for background perception workflows."""

from __future__ import annotations

import asyncio
import copy
import re
import tempfile
import uuid
from collections.abc import Coroutine
from pathlib import Path
from typing import Any, Callable, Mapping

from fastapi import APIRouter, File, Form, HTTPException, Request, Response, UploadFile, status

from perception.clients import (
    InvalidUpstreamResponseError,
    MediaTooLargeError,
    UnsafeMediaURLError,
    UpstreamConnectionError,
    UpstreamHTTPError,
    UpstreamTimeoutError,
)
from perception.clients.defect_detection import DefectDetectionClient
from perception.clients.video_recognition import VideoRecognitionClient
from perception.mission_identity import MissionIdentityError, MissionSnapshot
from perception.orchestrator import PerceptionOrchestrator
from perception.store import PerceptionJobNotFoundError, PerceptionJobStore


_ALLOWED_VIDEO_EXTENSIONS = {".avi", ".mkv", ".mov", ".mp4"}
_SAFE_DEFECT_MEDIA_RESOURCE = re.compile(
    r"^[A-Za-z0-9][A-Za-z0-9._-]{0,254}\.(?:jpe?g|png|bmp)$", re.IGNORECASE
)
_PUBLIC_ERROR_MESSAGES = {
    "all_target_frames_failed": "all target frames failed perception processing",
    "defect_detection_failed": "defect detection failed for this frame",
    "frame_download_failed": "target frame download failed",
    "identity_mismatch": "upstream business identity did not match the workflow",
    "invalid_frame": "video service returned an invalid target frame",
    "invalid_video_response": "video service returned an invalid response",
    "video_file_error": "temporary video file could not be read",
    "video_job_failed": "video recognition job failed",
    "video_poll_deadline_exceeded": "video recognition polling deadline exceeded",
    "video_workflow_failed": "video recognition service request failed",
}


OrchestratorFactory = Callable[..., PerceptionOrchestrator]
TaskScheduler = Callable[[Coroutine[Any, Any, None]], None]


class PerceptionAPI:
    """Own the perception router and its short-lived asyncio background tasks."""

    def __init__(
        self,
        *,
        store: PerceptionJobStore,
        video_client: VideoRecognitionClient,
        defect_client: DefectDetectionClient,
        mission_file: str | Path,
        temp_dir: str | Path | None = None,
        orchestrator_factory: OrchestratorFactory = PerceptionOrchestrator,
        task_scheduler: TaskScheduler | None = None,
    ) -> None:
        self.store = store
        self._video_client = video_client
        self._defect_client = defect_client
        self._mission_file = Path(mission_file)
        self._temp_dir = (
            Path(temp_dir)
            if temp_dir
            else Path(tempfile.gettempdir()) / "uav_perception_uploads"
        )
        self._orchestrator_factory = orchestrator_factory
        self._external_scheduler = task_scheduler
        self._tasks: set[asyncio.Task[None]] = set()
        self.router = APIRouter(prefix="/api/v1/perception", tags=["perception"])
        self._register_routes()

    async def aclose(self) -> None:
        tasks = list(self._tasks)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def _register_routes(self) -> None:
        @self.router.post("/jobs", status_code=status.HTTP_202_ACCEPTED)
        async def create_job(
            video: UploadFile = File(...),
            inspection_point_id: str = Form(...),
            video_id: str | None = Form(None),
        ) -> dict[str, Any]:
            snapshot = self._load_snapshot()
            try:
                snapshot.require_inspection_point(inspection_point_id)
            except MissionIdentityError as exc:
                raise self._http_error(
                    422,
                    "invalid_inspection_point_id",
                    "inspection_point_id is not authoritative for the current Mission",
                ) from exc

            suffix = Path(video.filename or "").suffix.lower()
            if suffix not in _ALLOWED_VIDEO_EXTENSIONS:
                raise self._http_error(
                    415,
                    "unsupported_video_type",
                    "video must use one of: .mp4, .avi, .mov, .mkv",
                )

            temp_path: Path | None = None
            try:
                content = await video.read()
                if not content:
                    raise self._http_error(
                        400, "empty_video", "uploaded video must not be empty"
                    )
                self._temp_dir.mkdir(parents=True, exist_ok=True)
                temp_path = self._temp_dir / f"{uuid.uuid4().hex}{suffix}"
                temp_path.write_bytes(content)
            except HTTPException:
                raise
            except OSError as exc:
                self._unlink_temp(temp_path)
                raise self._http_error(
                    500, "temporary_upload_failed", "video upload could not be staged"
                ) from exc
            finally:
                await video.close()

            job = self.store.create(
                mission_id=snapshot.mission_id,
                inspection_point_id=inspection_point_id,
                video_id=video_id,
            )
            workflow_job_id = job["workflow_job_id"]
            try:
                self._schedule(
                    self._run_workflow(
                        workflow_job_id=workflow_job_id,
                        snapshot=snapshot,
                        temp_path=temp_path,
                        inspection_point_id=inspection_point_id,
                        video_id=video_id,
                    )
                )
            except Exception as exc:
                self._unlink_temp(temp_path)
                self.store.store_error(
                    workflow_job_id,
                    code="background_task_start_failed",
                    message="perception background task could not be started",
                )
                raise self._http_error(
                    500,
                    "background_task_start_failed",
                    "perception background task could not be started",
                ) from exc
            return {"workflow_job_id": workflow_job_id, "status": "queued"}

        @self.router.get("/jobs/{workflow_job_id}")
        async def get_job(workflow_job_id: str) -> dict[str, Any]:
            job = self._get_job_or_404(workflow_job_id)
            return self._job_status_payload(job)

        @self.router.get("/jobs/{workflow_job_id}/result")
        async def get_result(workflow_job_id: str) -> dict[str, Any]:
            job = self._get_job_or_404(workflow_job_id)
            if job["status"] in {"queued", "running"}:
                raise self._http_error(
                    409,
                    "result_not_ready",
                    f"perception workflow is {job['status']}",
                    status=job["status"],
                )
            if job.get("result") is not None:
                return self._normalize_result_media_urls(job["result"])
            if job["status"] == "failed":
                raise self._http_error(
                    409,
                    "workflow_failed",
                    "perception workflow failed before producing a result",
                    status="failed",
                    error=job.get("error"),
                )
            raise self._http_error(
                500,
                "result_missing",
                "perception workflow finished without a stored result",
            )

        @self.router.get("/media/defect/{resource_id:path}")
        async def get_defect_media(resource_id: str, request: Request) -> Response:
            if request.url.query or not self._is_safe_defect_media_resource(
                resource_id
            ):
                raise self._http_error(
                    400,
                    "invalid_media_resource",
                    "defect media resource identifier is invalid",
                )
            try:
                media = await self._defect_client.download_result_media(resource_id)
            except UnsafeMediaURLError as exc:
                raise self._http_error(
                    400,
                    "invalid_media_resource",
                    "defect media resource identifier is invalid",
                ) from exc
            except UpstreamHTTPError as exc:
                if exc.status_code == 404:
                    raise self._http_error(
                        404,
                        "defect_media_not_found",
                        "annotated defect image was not found",
                    ) from exc
                raise self._http_error(
                    502,
                    "defect_media_upstream_error",
                    "defect media service returned an error",
                ) from exc
            except (UpstreamConnectionError, UpstreamTimeoutError) as exc:
                raise self._http_error(
                    503,
                    "defect_media_unavailable",
                    "defect media service is unavailable",
                ) from exc
            except (InvalidUpstreamResponseError, MediaTooLargeError) as exc:
                raise self._http_error(
                    502,
                    "invalid_defect_media",
                    "defect media service returned an invalid image",
                ) from exc
            return Response(
                content=media.content,
                media_type=media.content_type,
                headers={
                    "Cache-Control": "private, max-age=300",
                    "Content-Disposition": f'inline; filename="{media.filename}"',
                    "X-Content-Type-Options": "nosniff",
                },
            )

    def _load_snapshot(self) -> MissionSnapshot:
        try:
            return MissionSnapshot.from_file(self._mission_file)
        except FileNotFoundError as exc:
            raise self._http_error(
                503, "mission_unavailable", "current Mission is unavailable"
            ) from exc
        except (MissionIdentityError, OSError) as exc:
            raise self._http_error(
                503, "mission_invalid", "current Mission cannot be loaded"
            ) from exc

    async def _run_workflow(
        self,
        *,
        workflow_job_id: str,
        snapshot: MissionSnapshot,
        temp_path: Path,
        inspection_point_id: str,
        video_id: str | None,
    ) -> None:
        try:
            self.store.update(
                workflow_job_id, status="running", stage="starting", progress=5
            )
            orchestrator = self._orchestrator_factory(
                video_client=self._video_client,
                defect_client=self._defect_client,
                event_callback=lambda event, payload: self._handle_event(
                    workflow_job_id, event, payload
                ),
            )
            result = await orchestrator.run(
                mission_snapshot=snapshot,
                video_path=temp_path,
                inspection_point_id=inspection_point_id,
                video_id=video_id,
            )
            self.store.store_result(
                workflow_job_id, self._sanitize_result(result.to_dict())
            )
        except asyncio.CancelledError:
            raise
        except Exception:
            self.store.store_error(
                workflow_job_id,
                code="background_workflow_failed",
                message="perception background workflow failed",
            )
        finally:
            self._unlink_temp(temp_path)

    def _handle_event(
        self,
        workflow_job_id: str,
        event: str,
        payload: Mapping[str, Any],
    ) -> None:
        if event == "video_job_created":
            self.store.update(
                workflow_job_id,
                status="running",
                stage="video_running",
                progress=20,
                video_job_id=payload.get("job_id"),
            )
        elif event == "video_status":
            upstream_status = payload.get("status")
            progress = 50 if upstream_status == "completed" else 30
            self.store.update(
                workflow_job_id,
                status="running",
                stage="video_running",
                progress=progress,
            )
        elif event == "target_frames":
            self.store.update(
                workflow_job_id,
                status="running",
                stage="defect_running",
                progress=60,
            )
        elif event in {"frame_completed", "frame_failed"}:
            self.store.update(
                workflow_job_id,
                status="running",
                stage="defect_running",
                progress=80,
            )

    def _schedule(self, coroutine: Coroutine[Any, Any, None]) -> None:
        if self._external_scheduler is not None:
            self._external_scheduler(coroutine)
            return
        task = asyncio.create_task(coroutine)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    def _get_job_or_404(self, workflow_job_id: str) -> dict[str, Any]:
        try:
            return self.store.get(workflow_job_id)
        except PerceptionJobNotFoundError as exc:
            raise self._http_error(
                404, "workflow_job_not_found", "perception workflow job was not found"
            ) from exc

    @staticmethod
    def _job_status_payload(job: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "workflow_job_id": job["workflow_job_id"],
            "status": job["status"],
            "mission_id": job["mission_id"],
            "inspection_point_id": job["inspection_point_id"],
            "video_id": job.get("video_id"),
            "video_job_id": job.get("video_job_id"),
            "progress": job["progress"],
            "stage": job["stage"],
            "error": job.get("error"),
        }

    @classmethod
    def _sanitize_result(cls, result: dict[str, Any]) -> dict[str, Any]:
        safe = copy.deepcopy(result)
        for error in safe.get("errors") or []:
            cls._sanitize_error(error)
        for frame in safe.get("frames") or []:
            error = frame.get("error") if isinstance(frame, dict) else None
            if isinstance(error, dict):
                cls._sanitize_error(error)
        return safe

    @classmethod
    def _normalize_result_media_urls(
        cls, result: Mapping[str, Any]
    ) -> dict[str, Any]:
        safe = copy.deepcopy(dict(result))
        for frame in safe.get("frames") or []:
            if not isinstance(frame, dict):
                continue
            defect = frame.get("defect_detection")
            if not isinstance(defect, dict):
                continue
            upstream_url = defect.get("annotated_image_url")
            if not isinstance(upstream_url, str):
                continue
            match = re.fullmatch(r"/results/([^/?#]+)", upstream_url.strip())
            if not match:
                continue
            resource_id = match.group(1)
            if cls._is_safe_defect_media_resource(resource_id):
                defect["annotated_image_url"] = (
                    f"/api/v1/perception/media/defect/{resource_id}"
                )
        return safe

    @staticmethod
    def _is_safe_defect_media_resource(resource_id: str) -> bool:
        return bool(_SAFE_DEFECT_MEDIA_RESOURCE.fullmatch(resource_id))

    @staticmethod
    def _sanitize_error(error: dict[str, Any]) -> None:
        code = str(error.get("code") or "perception_error")
        error["code"] = code
        error["message"] = _PUBLIC_ERROR_MESSAGES.get(
            code, "perception processing failed"
        )

    @staticmethod
    def _unlink_temp(path: Path | None) -> None:
        if path is None:
            return
        try:
            path.unlink(missing_ok=True)
        except OSError:
            pass

    @staticmethod
    def _http_error(
        status_code: int,
        code: str,
        message: str,
        **extra: Any,
    ) -> HTTPException:
        detail = {"code": code, "message": message}
        detail.update(extra)
        return HTTPException(status_code=status_code, detail=detail)


__all__ = ["PerceptionAPI"]
