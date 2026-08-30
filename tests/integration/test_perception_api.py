from __future__ import annotations

import asyncio
import json
import threading
import time
from pathlib import Path
from typing import Any, Mapping

import pytest
import httpx
from fastapi import FastAPI
from fastapi.testclient import TestClient

from perception.clients.defect_detection import DefectDetectionClient
from perception.models import PerceptionWorkflowResult, WorkflowError
from perception.router import PerceptionAPI
from perception.store import PerceptionJobStore


class UnusedClient:
    pass


class ControlledOrchestratorFactory:
    def __init__(self, *, final_status: str = "completed") -> None:
        self.final_status = final_status
        self.started = threading.Event()
        self.release = threading.Event()
        self.temp_paths: list[Path] = []
        self.calls: list[dict[str, Any]] = []

    def __call__(self, **kwargs: Any) -> "ControlledOrchestrator":
        return ControlledOrchestrator(self, kwargs["event_callback"])


class ControlledOrchestrator:
    def __init__(
        self,
        factory: ControlledOrchestratorFactory,
        event_callback: Any,
    ) -> None:
        self.factory = factory
        self.event_callback = event_callback

    async def run(self, **kwargs: Any) -> PerceptionWorkflowResult:
        temp_path = Path(kwargs["video_path"])
        assert temp_path.is_file()
        assert temp_path.name != "flight.mp4"
        self.factory.temp_paths.append(temp_path)
        self.factory.calls.append(kwargs)
        self.event_callback(
            "video_job_created", {"job_id": "VR_JOB_1", "status": "queued"}
        )
        self.event_callback(
            "video_status", {"job_id": "VR_JOB_1", "status": "running"}
        )
        self.factory.started.set()
        while not self.factory.release.is_set():
            await asyncio.sleep(0.005)
        if self.factory.final_status == "failed":
            return PerceptionWorkflowResult(
                status="failed",
                mission_id=kwargs["mission_snapshot"].mission_id,
                inspection_point_id=kwargs["inspection_point_id"],
                video_job_id="VR_JOB_1",
                video_id=kwargs.get("video_id"),
                errors=(WorkflowError("video_job_failed", "secret local path D:\\x"),),
            )
        if self.factory.final_status == "completed_with_errors":
            return PerceptionWorkflowResult(
                status="completed_with_errors",
                mission_id=kwargs["mission_snapshot"].mission_id,
                inspection_point_id=kwargs["inspection_point_id"],
                video_job_id="VR_JOB_1",
                video_id=kwargs.get("video_id"),
                target_frame_count=2,
                errors=(WorkflowError("defect_detection_failed", "private detail"),),
            )
        self.event_callback("target_frames", {"count": 1})
        return PerceptionWorkflowResult(
            status="completed",
            mission_id=kwargs["mission_snapshot"].mission_id,
            inspection_point_id=kwargs["inspection_point_id"],
            video_job_id="VR_JOB_1",
            video_id=kwargs.get("video_id"),
            target_frame_count=1,
        )


def write_mission(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "inspection_points": [
                    {"point_id": "IP_00001"},
                    {"point_id": "IP_00012"},
                ],
                "image_inspection_overlay": [{"id": "IP_0012"}],
            }
        ),
        encoding="utf-8",
    )


def make_app(
    tmp_path: Path,
    *,
    factory: ControlledOrchestratorFactory | None = None,
    mission_exists: bool = True,
    defect_client: Any | None = None,
) -> tuple[FastAPI, PerceptionAPI, ControlledOrchestratorFactory]:
    mission_file = tmp_path / "mission.json"
    if mission_exists:
        write_mission(mission_file)
    controlled = factory or ControlledOrchestratorFactory()
    api = PerceptionAPI(
        store=PerceptionJobStore(),
        video_client=UnusedClient(),
        defect_client=defect_client or UnusedClient(),
        mission_file=mission_file,
        temp_dir=tmp_path / "uploads",
        orchestrator_factory=controlled,
    )
    test_app = FastAPI()
    test_app.include_router(api.router)

    @test_app.on_event("shutdown")
    async def shutdown() -> None:
        await api.aclose()

    return test_app, api, controlled


def post_job(client: TestClient, point_id: str = "IP_00012"):
    return client.post(
        "/api/v1/perception/jobs",
        files={"video": ("flight.mp4", b"video-bytes", "video/mp4")},
        data={"inspection_point_id": point_id, "video_id": "VIDEO_1"},
    )


def wait_for_status(
    client: TestClient,
    workflow_job_id: str,
    expected: str,
    timeout: float = 2.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = client.get(f"/api/v1/perception/jobs/{workflow_job_id}")
        body = response.json()
        if body["status"] == expected:
            return body
        time.sleep(0.01)
    pytest.fail(f"job did not reach {expected}")


def test_post_returns_202_then_running_completed_and_cleans_upload(
    tmp_path: Path,
) -> None:
    app, _api, factory = make_app(tmp_path)
    with TestClient(app) as client:
        response = post_job(client)
        assert response.status_code == 202
        assert response.json()["status"] == "queued"
        workflow_job_id = response.json()["workflow_job_id"]
        assert factory.started.wait(1)

        running = client.get(
            f"/api/v1/perception/jobs/{workflow_job_id}"
        ).json()
        assert running["status"] == "running"
        assert running["stage"] == "video_running"
        assert running["video_job_id"] == "VR_JOB_1"
        not_ready = client.get(
            f"/api/v1/perception/jobs/{workflow_job_id}/result"
        )
        assert not_ready.status_code == 409
        assert not_ready.json()["detail"]["code"] == "result_not_ready"
        staged_path = factory.temp_paths[0]
        assert staged_path.is_file()
        assert staged_path.suffix == ".mp4"

        factory.release.set()
        completed = wait_for_status(client, workflow_job_id, "completed")
        assert completed["progress"] == 100
        result = client.get(
            f"/api/v1/perception/jobs/{workflow_job_id}/result"
        )
        assert result.status_code == 200
        assert result.json()["status"] == "completed"
        assert result.json()["target_frame_count"] == 1
        assert not staged_path.exists()


def test_non_authoritative_overlay_id_is_rejected_before_background_work(
    tmp_path: Path,
) -> None:
    app, _api, factory = make_app(tmp_path)
    with TestClient(app) as client:
        response = post_job(client, "IP_0012")
        assert response.status_code == 422
        assert response.json()["detail"]["code"] == "invalid_inspection_point_id"
        assert not factory.started.is_set()
        assert not (tmp_path / "uploads").exists()


def test_missing_mission_is_structured_503(tmp_path: Path) -> None:
    app, _api, _factory = make_app(tmp_path, mission_exists=False)
    with TestClient(app) as client:
        response = post_job(client)
        assert response.status_code == 503
        assert response.json()["detail"] == {
            "code": "mission_unavailable",
            "message": "current Mission is unavailable",
        }


def test_unknown_job_is_404(tmp_path: Path) -> None:
    app, _api, _factory = make_app(tmp_path)
    with TestClient(app) as client:
        response = client.get("/api/v1/perception/jobs/missing")
        assert response.status_code == 404
        assert response.json()["detail"]["code"] == "workflow_job_not_found"


def test_failed_workflow_has_final_result_and_sanitized_error(tmp_path: Path) -> None:
    factory = ControlledOrchestratorFactory(final_status="failed")
    app, _api, factory = make_app(tmp_path, factory=factory)
    with TestClient(app) as client:
        response = post_job(client)
        workflow_job_id = response.json()["workflow_job_id"]
        assert factory.started.wait(1)
        factory.release.set()
        failed = wait_for_status(client, workflow_job_id, "failed")
        assert failed["error"]["code"] == "video_job_failed"
        assert "D:\\x" not in failed["error"]["message"]
        result = client.get(
            f"/api/v1/perception/jobs/{workflow_job_id}/result"
        )
        assert result.status_code == 200
        assert result.json()["status"] == "failed"
        assert result.json()["errors"][0]["message"] == "video recognition job failed"


def test_completed_with_errors_is_queryable_as_a_final_result(tmp_path: Path) -> None:
    factory = ControlledOrchestratorFactory(final_status="completed_with_errors")
    app, _api, factory = make_app(tmp_path, factory=factory)
    with TestClient(app) as client:
        response = post_job(client)
        workflow_job_id = response.json()["workflow_job_id"]
        assert factory.started.wait(1)
        factory.release.set()
        partial = wait_for_status(client, workflow_job_id, "completed_with_errors")
        assert partial["error"] == {
            "code": "defect_detection_failed",
            "message": "defect detection failed for this frame",
            "frame_id": None,
        }
        result = client.get(
            f"/api/v1/perception/jobs/{workflow_job_id}/result"
        )
        assert result.status_code == 200
        assert result.json()["status"] == "completed_with_errors"


def test_main_app_existing_debug_route_remains_available() -> None:
    from app import app as main_app

    with TestClient(main_app) as client:
        response = client.get("/debug/ping")
        assert response.status_code == 200
        assert response.json() == {"ok": True, "app": "current app.py"}


def make_defect_media_client(
    handler: Any,
) -> tuple[DefectDetectionClient, httpx.AsyncClient]:
    http_client = httpx.AsyncClient(transport=httpx.MockTransport(handler))
    return (
        DefectDetectionClient(
            base_url="http://127.0.0.1:8003",
            client=http_client,
        ),
        http_client,
    )


@pytest.mark.parametrize(
    ("resource_id", "content_type"),
    [("annotated-1.jpg", "image/jpeg"), ("annotated_2.png", "image/png")],
)
def test_defect_media_proxy_returns_safe_images(
    tmp_path: Path,
    resource_id: str,
    content_type: str,
) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url == httpx.URL(
            f"http://127.0.0.1:8003/results/{resource_id}"
        )
        return httpx.Response(
            200,
            content=b"image-bytes",
            headers={"content-type": content_type},
        )

    defect_client, http_client = make_defect_media_client(handler)
    app, _api, _factory = make_app(tmp_path, defect_client=defect_client)
    with TestClient(app) as client:
        response = client.get(
            f"/api/v1/perception/media/defect/{resource_id}"
        )
        assert response.status_code == 200
        assert response.content == b"image-bytes"
        assert response.headers["content-type"] == content_type
    asyncio.run(http_client.aclose())


@pytest.mark.parametrize(
    "unsafe_path",
    [
        "%2E%2E%2Fsecret.jpg",
        "%2E%2E%5Csecret.jpg",
        "http%3A%2F%2F127.0.0.1%3A8003%2Fresults%2Fx.jpg",
        "safe.jpg%3Fevil%3D1",
    ],
)
def test_defect_media_proxy_rejects_unsafe_resource_ids(
    tmp_path: Path,
    unsafe_path: str,
) -> None:
    defect_client, http_client = make_defect_media_client(
        lambda request: pytest.fail("unsafe resource must not reach 8003")
    )
    app, _api, _factory = make_app(tmp_path, defect_client=defect_client)
    with TestClient(app) as client:
        response = client.get(
            f"/api/v1/perception/media/defect/{unsafe_path}"
        )
        assert response.status_code in {400, 404}
    asyncio.run(http_client.aclose())


def test_defect_media_proxy_rejects_query_injection(tmp_path: Path) -> None:
    defect_client, http_client = make_defect_media_client(
        lambda request: pytest.fail("query injection must not reach 8003")
    )
    app, _api, _factory = make_app(tmp_path, defect_client=defect_client)
    with TestClient(app) as client:
        response = client.get(
            "/api/v1/perception/media/defect/safe.jpg?upstream=/other"
        )
        assert response.status_code == 400
        assert response.json()["detail"]["code"] == "invalid_media_resource"
    asyncio.run(http_client.aclose())


def test_defect_media_proxy_maps_upstream_404(tmp_path: Path) -> None:
    defect_client, http_client = make_defect_media_client(
        lambda request: httpx.Response(404, json={"detail": "missing"})
    )
    app, _api, _factory = make_app(tmp_path, defect_client=defect_client)
    with TestClient(app) as client:
        response = client.get(
            "/api/v1/perception/media/defect/missing.jpg"
        )
        assert response.status_code == 404
        assert response.json()["detail"] == {
            "code": "defect_media_not_found",
            "message": "annotated defect image was not found",
        }
    asyncio.run(http_client.aclose())


def test_defect_media_proxy_maps_timeout_to_structured_503(tmp_path: Path) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("timeout", request=request)

    defect_client, http_client = make_defect_media_client(handler)
    app, _api, _factory = make_app(tmp_path, defect_client=defect_client)
    with TestClient(app) as client:
        response = client.get(
            "/api/v1/perception/media/defect/slow.jpg"
        )
        assert response.status_code == 503
        assert response.json()["detail"]["code"] == "defect_media_unavailable"
    asyncio.run(http_client.aclose())


def test_defect_media_proxy_rejects_non_image_content_type(tmp_path: Path) -> None:
    defect_client, http_client = make_defect_media_client(
        lambda request: httpx.Response(
            200,
            content=b"not-an-image",
            headers={"content-type": "text/html"},
        )
    )
    app, _api, _factory = make_app(tmp_path, defect_client=defect_client)
    with TestClient(app) as client:
        response = client.get(
            "/api/v1/perception/media/defect/not-image.jpg"
        )
        assert response.status_code == 502
        assert response.json()["detail"]["code"] == "invalid_defect_media"
    asyncio.run(http_client.aclose())


def test_result_normalizes_8003_relative_annotation_url_at_api_boundary(
    tmp_path: Path,
) -> None:
    app, api, _factory = make_app(tmp_path)
    job = api.store.create(
        mission_id="M_RUNTIME",
        inspection_point_id="IP_00012",
    )
    api.store.store_result(
        job["workflow_job_id"],
        {
            "status": "completed",
            "mission_id": "M_RUNTIME",
            "inspection_point_id": "IP_00012",
            "target_frame_count": 1,
            "frames": [
                {
                    "frame_id": "FRAME_1",
                    "defect_detection": {
                        "annotated_image_url": "/results/annotated.jpg"
                    },
                }
            ],
            "errors": [],
        },
    )
    with TestClient(app) as client:
        response = client.get(
            f"/api/v1/perception/jobs/{job['workflow_job_id']}/result"
        )
        assert response.status_code == 200
        assert response.json()["frames"][0]["defect_detection"][
            "annotated_image_url"
        ] == "/api/v1/perception/media/defect/annotated.jpg"

    stored = api.store.get(job["workflow_job_id"])["result"]
    assert stored["frames"][0]["defect_detection"][
        "annotated_image_url"
    ] == "/results/annotated.jpg"
