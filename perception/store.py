"""Process-local job state for the Phase 3 perception API."""

from __future__ import annotations

import copy
import threading
import uuid
from datetime import datetime, timezone
from typing import Any


class PerceptionJobNotFoundError(KeyError):
    """The requested workflow job does not exist in this process."""


class PerceptionJobStore:
    """Small thread-safe in-memory store; all state is lost on process restart."""

    def __init__(self) -> None:
        self._jobs: dict[str, dict[str, Any]] = {}
        self._lock = threading.RLock()

    def create(
        self,
        *,
        mission_id: str,
        mission_sha256: str,
        inspection_point_id: str,
        video_id: str | None = None,
    ) -> dict[str, Any]:
        now = self._now()
        workflow_job_id = f"perception_job_{uuid.uuid4().hex}"
        job = {
            "workflow_job_id": workflow_job_id,
            "status": "queued",
            "stage": "queued",
            "progress": 0,
            "mission_id": mission_id,
            "mission_sha256": mission_sha256,
            "inspection_point_id": inspection_point_id,
            "video_id": video_id,
            "video_job_id": None,
            "error": None,
            "result": None,
            "created_at": now,
            "updated_at": now,
        }
        with self._lock:
            self._jobs[workflow_job_id] = job
        return copy.deepcopy(job)

    def get(self, workflow_job_id: str) -> dict[str, Any]:
        with self._lock:
            try:
                return copy.deepcopy(self._jobs[workflow_job_id])
            except KeyError as exc:
                raise PerceptionJobNotFoundError(workflow_job_id) from exc

    def update(self, workflow_job_id: str, **changes: Any) -> dict[str, Any]:
        with self._lock:
            try:
                job = self._jobs[workflow_job_id]
            except KeyError as exc:
                raise PerceptionJobNotFoundError(workflow_job_id) from exc
            job.update(changes)
            job["updated_at"] = self._now()
            return copy.deepcopy(job)

    def store_result(
        self, workflow_job_id: str, result: dict[str, Any]
    ) -> dict[str, Any]:
        status = result.get("status")
        if status not in {"completed", "completed_with_errors", "failed"}:
            raise ValueError(f"cannot store non-final workflow status: {status!r}")
        errors = result.get("errors")
        first_error = errors[0] if isinstance(errors, list) and errors else None
        return self.update(
            workflow_job_id,
            status=status,
            stage="finished",
            progress=100,
            video_job_id=result.get("video_job_id"),
            video_id=result.get("video_id"),
            error=(
                first_error
                if status in {"completed_with_errors", "failed"}
                else None
            ),
            result=copy.deepcopy(result),
        )

    def store_error(
        self,
        workflow_job_id: str,
        *,
        code: str,
        message: str,
    ) -> dict[str, Any]:
        return self.update(
            workflow_job_id,
            status="failed",
            stage="finished",
            progress=100,
            error={"code": code, "message": message},
            result=None,
        )

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()


__all__ = ["PerceptionJobNotFoundError", "PerceptionJobStore"]
