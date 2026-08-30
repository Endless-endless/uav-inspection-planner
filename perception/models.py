"""Small result models for the Phase 2 perception workflow."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class WorkflowError:
    code: str
    message: str
    frame_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "message": self.message,
            "frame_id": self.frame_id,
        }


@dataclass(frozen=True)
class FrameResult:
    frame_id: str | None
    timestamp_ms: int | None
    source_image_url: str | None
    status: str
    defect_detection: dict[str, Any] | None = None
    error: WorkflowError | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "timestamp_ms": self.timestamp_ms,
            "source_image_url": self.source_image_url,
            "status": self.status,
            "defect_detection": self.defect_detection,
            "error": self.error.to_dict() if self.error else None,
        }


@dataclass(frozen=True)
class PerceptionWorkflowResult:
    status: str
    mission_id: str
    mission_sha256: str
    inspection_point_id: str
    video_job_id: str | None = None
    video_id: str | None = None
    target_frame_count: int = 0
    frames: tuple[FrameResult, ...] = field(default_factory=tuple)
    errors: tuple[WorkflowError, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "mission_id": self.mission_id,
            "mission_sha256": self.mission_sha256,
            "inspection_point_id": self.inspection_point_id,
            "video_job_id": self.video_job_id,
            "video_id": self.video_id,
            "target_frame_count": self.target_frame_count,
            "frames": [frame.to_dict() for frame in self.frames],
            "errors": [error.to_dict() for error in self.errors],
        }


__all__ = ["FrameResult", "PerceptionWorkflowResult", "WorkflowError"]
