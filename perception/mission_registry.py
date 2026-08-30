"""Process-local registry for exact runtime Mission snapshots."""

from __future__ import annotations

import copy
import threading
from typing import Any, Mapping

from perception.mission_identity import MissionSnapshot


class RuntimeMissionUnavailableError(KeyError):
    """The requested Mission is not registered in this process."""


class RuntimeMissionIdentityMismatchError(ValueError):
    """A Mission id exists, but its hash does not match the request."""


class RuntimeMissionRegistry:
    """Thread-safe in-memory storage for immutable Mission snapshots."""

    def __init__(self) -> None:
        self._by_id: dict[str, MissionSnapshot] = {}
        self._id_by_hash: dict[str, str] = {}
        self._current_id: str | None = None
        self._lock = threading.RLock()

    def register(
        self, mission: Mapping[str, Any], *, make_current: bool = True
    ) -> MissionSnapshot:
        snapshot = MissionSnapshot.from_dict(mission)
        with self._lock:
            existing = self._by_id.get(snapshot.mission_id)
            if existing and existing.mission_sha256 != snapshot.mission_sha256:
                raise RuntimeMissionIdentityMismatchError(
                    f"mission_id {snapshot.mission_id!r} is already registered with a different hash"
                )
            self._by_id[snapshot.mission_id] = copy.deepcopy(snapshot)
            self._id_by_hash[snapshot.mission_sha256] = snapshot.mission_id
            if make_current:
                self._current_id = snapshot.mission_id
        return copy.deepcopy(snapshot)

    def get(self, mission_id: str, mission_sha256: str) -> MissionSnapshot:
        with self._lock:
            snapshot = self._by_id.get(mission_id)
            if snapshot is None:
                raise RuntimeMissionUnavailableError(mission_id)
            if snapshot.mission_sha256 != mission_sha256:
                raise RuntimeMissionIdentityMismatchError(mission_id)
            return copy.deepcopy(snapshot)

    def get_by_hash(self, mission_sha256: str) -> MissionSnapshot:
        with self._lock:
            mission_id = self._id_by_hash.get(mission_sha256)
            if mission_id is None:
                raise RuntimeMissionUnavailableError(mission_sha256)
            return copy.deepcopy(self._by_id[mission_id])

    def current(self) -> MissionSnapshot:
        with self._lock:
            if self._current_id is None:
                raise RuntimeMissionUnavailableError("current")
            return copy.deepcopy(self._by_id[self._current_id])

    def clear(self) -> None:
        with self._lock:
            self._by_id.clear()
            self._id_by_hash.clear()
            self._current_id = None


__all__ = [
    "RuntimeMissionIdentityMismatchError",
    "RuntimeMissionRegistry",
    "RuntimeMissionUnavailableError",
]
