"""Mission snapshot and authoritative inspection-point identity handling."""

from __future__ import annotations

import copy
import hashlib
import json
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


class MissionIdentityError(ValueError):
    """Mission identity data is missing, duplicated, or inconsistent."""


@dataclass(frozen=True)
class MissionIdentity:
    mission_id: str
    authoritative_inspection_point_ids: frozenset[str]
    mission_sha256: str

    def require_inspection_point(self, inspection_point_id: str) -> None:
        if inspection_point_id not in self.authoritative_inspection_point_ids:
            raise MissionIdentityError(
                f"inspection_point_id {inspection_point_id!r} is not present in "
                "mission['inspection_points'][].point_id"
            )


@dataclass(frozen=True)
class MissionSnapshot:
    mission: Mapping[str, Any]
    identity: MissionIdentity
    source_path: Path | None = None

    @property
    def mission_id(self) -> str:
        return self.identity.mission_id

    @property
    def authoritative_inspection_point_ids(self) -> frozenset[str]:
        return self.identity.authoritative_inspection_point_ids

    @property
    def mission_sha256(self) -> str:
        return self.identity.mission_sha256

    def require_inspection_point(self, inspection_point_id: str) -> None:
        self.identity.require_inspection_point(inspection_point_id)

    @classmethod
    def from_file(
        cls,
        mission_file: str | Path,
        *,
        runtime_mission_id: str | None = None,
    ) -> "MissionSnapshot":
        path = Path(mission_file).resolve()
        raw = path.read_bytes()
        try:
            mission = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise MissionIdentityError(f"invalid Mission JSON: {path}") from exc
        return cls._build(
            mission,
            mission_sha256=hashlib.sha256(raw).hexdigest(),
            runtime_mission_id=runtime_mission_id,
            source_path=path,
        )

    @classmethod
    def from_dict(
        cls,
        mission: Mapping[str, Any],
        *,
        runtime_mission_id: str | None = None,
    ) -> "MissionSnapshot":
        canonical = json.dumps(
            mission, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        return cls._build(
            mission,
            mission_sha256=hashlib.sha256(canonical).hexdigest(),
            runtime_mission_id=runtime_mission_id,
            source_path=None,
        )

    @classmethod
    def _build(
        cls,
        mission: Mapping[str, Any],
        *,
        mission_sha256: str,
        runtime_mission_id: str | None,
        source_path: Path | None,
    ) -> "MissionSnapshot":
        if not isinstance(mission, Mapping):
            raise MissionIdentityError("Mission root must be a JSON object")
        inspection_points = mission.get("inspection_points")
        if not isinstance(inspection_points, list):
            raise MissionIdentityError("Mission inspection_points must be an array")

        point_ids: list[str] = []
        for index, point in enumerate(inspection_points):
            if not isinstance(point, Mapping):
                raise MissionIdentityError(
                    f"inspection_points[{index}] must be an object"
                )
            point_id = point.get("point_id")
            if not isinstance(point_id, str) or not point_id:
                raise MissionIdentityError(
                    f"inspection_points[{index}].point_id must be a non-empty string"
                )
            point_ids.append(point_id)
        if len(point_ids) != len(set(point_ids)):
            raise MissionIdentityError(
                "Mission inspection_points contain duplicate authoritative point_id values"
            )

        if runtime_mission_id is not None and not runtime_mission_id:
            raise MissionIdentityError("runtime mission_id must be a non-empty string")
        mission_id = (
            runtime_mission_id
            if runtime_mission_id is not None
            else cls._existing_mission_id(mission)
        )
        if mission_id is None:
            mission_id = f"mission_rt_{uuid.uuid4().hex}"
        if not isinstance(mission_id, str) or not mission_id:
            raise MissionIdentityError("runtime mission_id must be a non-empty string")

        return cls(
            mission=copy.deepcopy(dict(mission)),
            identity=MissionIdentity(
                mission_id=mission_id,
                authoritative_inspection_point_ids=frozenset(point_ids),
                mission_sha256=mission_sha256,
            ),
            source_path=source_path,
        )

    @staticmethod
    def _existing_mission_id(mission: Mapping[str, Any]) -> str | None:
        top_level = mission.get("mission_id")
        if isinstance(top_level, str) and top_level:
            return top_level
        metadata = mission.get("metadata")
        if isinstance(metadata, Mapping):
            metadata_id = metadata.get("mission_id")
            if isinstance(metadata_id, str) and metadata_id:
                return metadata_id
        return None


__all__ = ["MissionIdentity", "MissionIdentityError", "MissionSnapshot"]
