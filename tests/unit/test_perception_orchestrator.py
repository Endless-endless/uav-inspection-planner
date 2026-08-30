from __future__ import annotations

import asyncio
import json
from collections import deque
from pathlib import Path
from typing import Any

import pytest

from perception.clients.video_recognition import DownloadedMedia
from perception.mission_identity import MissionIdentityError, MissionSnapshot
from perception.orchestrator import PerceptionOrchestrator


def run(coro: Any) -> Any:
    return asyncio.run(coro)


def mission_data(*, mission_id: str | None = None) -> dict[str, Any]:
    mission: dict[str, Any] = {
        "inspection_points": [
            {"point_id": "IP_00001", "x": 10, "y": 20},
            {"point_id": "IP_00012", "x": 30, "y": 40},
        ],
        "image_inspection_overlay": [
            {"id": "IP_0001", "x": 10, "y": 20},
            {"id": "IP_0012", "x": 30, "y": 40},
        ],
    }
    if mission_id is not None:
        mission["mission_id"] = mission_id
    return mission


def target_frame(
    frame_id: str = "FRAME_1", **changes: Any
) -> dict[str, Any]:
    frame = {
        "mission_id": "mission_rt_test",
        "inspection_point_id": "IP_00012",
        "video_id": "VIDEO_1",
        "frame_id": frame_id,
        "timestamp_ms": 750,
        "image_url": f"/media/{frame_id}.jpg",
    }
    frame.update(changes)
    return frame


class FakeVideoClient:
    def __init__(
        self,
        jobs: list[dict[str, Any]],
        *,
        download_failures: set[str] | None = None,
    ) -> None:
        self.jobs = deque(jobs)
        self.download_failures = download_failures or set()
        self.create_calls: list[dict[str, Any]] = []
        self.get_calls: list[str] = []
        self.download_calls: list[str] = []

    async def create_job(self, **kwargs: Any) -> dict[str, Any]:
        self.create_calls.append(kwargs)
        return {
            "job_id": "VR_JOB_1",
            "status": "queued",
            "mission_id": kwargs["mission_id"],
            "inspection_point_id": kwargs["inspection_point_id"],
            "video_id": kwargs.get("video_id") or "VIDEO_1",
        }

    async def get_job(self, job_id: str) -> dict[str, Any]:
        self.get_calls.append(job_id)
        return self.jobs.popleft()

    async def download_media(self, image_url: str) -> DownloadedMedia:
        self.download_calls.append(image_url)
        if image_url in self.download_failures:
            raise RuntimeError("download failed")
        return DownloadedMedia(b"image", Path(image_url).name, "image/jpeg")


class FakeDefectClient:
    def __init__(self, *, failures: set[str] | None = None) -> None:
        self.failures = failures or set()
        self.calls: list[dict[str, Any]] = []

    async def detect_image(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(kwargs)
        if kwargs["frame_id"] in self.failures:
            raise RuntimeError("defect failed")
        return {
            "success": True,
            "mission_id": kwargs["mission_id"],
            "inspection_point_id": kwargs["inspection_point_id"],
            "frame_id": kwargs["frame_id"],
            "timestamp_ms": kwargs["timestamp_ms"],
            "detection_count": 1,
            "detections": [{"class_name": "bird_nest"}],
            "annotated_image_url": "/annotated.jpg",
        }


async def no_sleep(seconds: float) -> None:
    return None


def make_orchestrator(
    video: FakeVideoClient,
    defect: FakeDefectClient,
    **kwargs: Any,
) -> PerceptionOrchestrator:
    return PerceptionOrchestrator(
        video_client=video,
        defect_client=defect,
        poll_interval_seconds=0,
        sleep=no_sleep,
        **kwargs,
    )


def write_video(tmp_path: Path) -> Path:
    path = tmp_path / "flight.mp4"
    path.write_bytes(b"video")
    return path


def completed_job(frames: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "job_id": "VR_JOB_1",
        "status": "completed",
        "video_id": "VIDEO_1",
        "target_frames": frames,
    }


def test_mission_without_id_gets_runtime_id_and_sha(tmp_path: Path) -> None:
    mission_file = tmp_path / "mission.json"
    mission_file.write_text(json.dumps(mission_data()), encoding="utf-8")
    snapshot = MissionSnapshot.from_file(mission_file)
    assert snapshot.mission_id.startswith("mission_rt_")
    assert len(snapshot.mission_sha256) == 64
    assert snapshot.authoritative_inspection_point_ids == {
        "IP_00001",
        "IP_00012",
    }


def test_authoritative_id_is_exact_and_overlay_id_is_rejected() -> None:
    snapshot = MissionSnapshot.from_dict(mission_data())
    snapshot.require_inspection_point("IP_00012")
    with pytest.raises(MissionIdentityError, match="inspection_points"):
        snapshot.require_inspection_point("IP_0012")


def test_explicit_empty_runtime_mission_id_is_rejected() -> None:
    with pytest.raises(MissionIdentityError, match="runtime mission_id"):
        MissionSnapshot.from_dict(mission_data(), runtime_mission_id="")


def test_invalid_point_fails_before_video_service_call(tmp_path: Path) -> None:
    video = FakeVideoClient([completed_job([])])
    defect = FakeDefectClient()
    orchestrator = make_orchestrator(video, defect)
    with pytest.raises(MissionIdentityError):
        run(
            orchestrator.run(
                mission_snapshot=MissionSnapshot.from_dict(mission_data()),
                video_path=write_video(tmp_path),
                inspection_point_id="IP_0012",
            )
        )
    assert video.create_calls == []


def test_queued_running_completed_sends_frame_to_defect(tmp_path: Path) -> None:
    video = FakeVideoClient(
        [
            {"status": "queued"},
            {"status": "running"},
            completed_job([target_frame()]),
        ]
    )
    defect = FakeDefectClient()
    events: list[tuple[str, dict[str, Any]]] = []
    orchestrator = make_orchestrator(
        video, defect, event_callback=lambda event, data: events.append((event, dict(data)))
    )
    result = run(
        orchestrator.run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
            video_id="VIDEO_1",
        )
    )
    assert result.status == "completed"
    assert [event[1]["status"] for event in events if event[0] == "video_status"] == [
        "queued",
        "running",
        "completed",
    ]
    assert defect.calls[0]["mission_id"] == "mission_rt_test"
    assert defect.calls[0]["inspection_point_id"] == "IP_00012"
    assert defect.calls[0]["frame_id"] == "FRAME_1"
    assert defect.calls[0]["timestamp_ms"] == 750


def test_video_failed_does_not_call_defect(tmp_path: Path) -> None:
    video = FakeVideoClient(
        [{"status": "failed", "error": {"message": "inference failed"}}]
    )
    defect = FakeDefectClient()
    result = run(
        make_orchestrator(video, defect).run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
        )
    )
    assert result.status == "failed"
    assert result.errors[0].code == "video_job_failed"
    assert defect.calls == []


@pytest.mark.parametrize(
    "changes, expected_message",
    [
        ({"mission_id": "OTHER"}, "mission_id"),
        ({"inspection_point_id": "IP_00001"}, "inspection_point_id"),
    ],
)
def test_frame_identity_mismatch_is_not_corrected(
    tmp_path: Path, changes: dict[str, Any], expected_message: str
) -> None:
    video = FakeVideoClient([completed_job([target_frame(**changes)])])
    defect = FakeDefectClient()
    result = run(
        make_orchestrator(video, defect).run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
        )
    )
    assert result.status == "failed"
    assert result.frames[0].status == "identity_mismatch"
    assert expected_message in result.errors[0].message
    assert defect.calls == []


@pytest.mark.parametrize(
    "changes, expected_message",
    [
        ({"frame_id": ""}, "frame_id"),
        ({"timestamp_ms": -1}, "timestamp_ms"),
        ({"timestamp_ms": "750"}, "timestamp_ms"),
    ],
)
def test_invalid_frame_fields_are_rejected(
    tmp_path: Path, changes: dict[str, Any], expected_message: str
) -> None:
    video = FakeVideoClient([completed_job([target_frame(**changes)])])
    defect = FakeDefectClient()
    result = run(
        make_orchestrator(video, defect).run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
        )
    )
    assert result.status == "failed"
    assert result.frames[0].status == "invalid_frame"
    assert expected_message in result.errors[0].message
    assert defect.calls == []


def test_download_failure_does_not_stop_other_frames(tmp_path: Path) -> None:
    frames = [target_frame("FRAME_1"), target_frame("FRAME_2")]
    video = FakeVideoClient(
        [completed_job(frames)], download_failures={"/media/FRAME_1.jpg"}
    )
    defect = FakeDefectClient()
    result = run(
        make_orchestrator(video, defect).run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
        )
    )
    assert result.status == "completed_with_errors"
    assert [frame.status for frame in result.frames] == [
        "download_failed",
        "completed",
    ]
    assert [call["frame_id"] for call in defect.calls] == ["FRAME_2"]


def test_defect_failure_does_not_stop_other_frames(tmp_path: Path) -> None:
    frames = [target_frame("FRAME_1"), target_frame("FRAME_2")]
    video = FakeVideoClient([completed_job(frames)])
    defect = FakeDefectClient(failures={"FRAME_1"})
    result = run(
        make_orchestrator(video, defect).run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
        )
    )
    assert result.status == "completed_with_errors"
    assert [frame.status for frame in result.frames] == [
        "defect_failed",
        "completed",
    ]
    assert [call["frame_id"] for call in defect.calls] == ["FRAME_1", "FRAME_2"]


def test_poll_deadline_is_bounded_and_defect_is_not_called(tmp_path: Path) -> None:
    clock_values = iter([0.0, 0.0, 2.0])
    video = FakeVideoClient([{"status": "queued"}])
    defect = FakeDefectClient()
    orchestrator = make_orchestrator(
        video,
        defect,
        poll_deadline_seconds=1.0,
        clock=lambda: next(clock_values),
    )
    result = run(
        orchestrator.run(
            mission_snapshot=MissionSnapshot.from_dict(
                mission_data(), runtime_mission_id="mission_rt_test"
            ),
            video_path=write_video(tmp_path),
            inspection_point_id="IP_00012",
        )
    )
    assert result.status == "failed"
    assert result.errors[0].code == "video_poll_deadline_exceeded"
    assert defect.calls == []
