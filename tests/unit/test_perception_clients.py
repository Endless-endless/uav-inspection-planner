from __future__ import annotations

import asyncio
import json
from email.parser import BytesParser
from email.policy import default
from typing import Any, Callable

import httpx
import pytest

from perception.clients import (
    IdentityMismatchError,
    InvalidUpstreamResponseError,
    MediaTooLargeError,
    UnsafeMediaURLError,
    UpstreamHTTPError,
    UpstreamTimeoutError,
)
from perception.clients.defect_detection import DefectDetectionClient
from perception.clients.video_recognition import VideoRecognitionClient


def run(coro: Any) -> Any:
    return asyncio.run(coro)


def async_client(handler: Callable[[httpx.Request], httpx.Response]) -> httpx.AsyncClient:
    return httpx.AsyncClient(transport=httpx.MockTransport(handler))


def json_response(status: int, payload: Any) -> httpx.Response:
    return httpx.Response(status, json=payload)


def multipart_parts(request: httpx.Request) -> dict[str, Any]:
    content_type = request.headers["content-type"]
    message = BytesParser(policy=default).parsebytes(
        f"Content-Type: {content_type}\r\nMIME-Version: 1.0\r\n\r\n".encode()
        + request.content
    )
    parts: dict[str, Any] = {}
    for part in message.iter_parts():
        name = part.get_param("name", header="content-disposition")
        filename = part.get_filename()
        value = part.get_payload(decode=True)
        parts[name] = (filename, value, part.get_content_type()) if filename else value.decode()
    return parts


def test_video_health_success() -> None:
    client = async_client(lambda request: json_response(200, {"status": "ok"}))
    api = VideoRecognitionClient(client=client)
    assert run(api.health()) == {"status": "ok"}
    run(client.aclose())


def test_video_health_http_failure() -> None:
    client = async_client(lambda request: json_response(503, {"error": "down"}))
    api = VideoRecognitionClient(client=client)
    with pytest.raises(UpstreamHTTPError, match="HTTP 503"):
        run(api.health())
    run(client.aclose())


def test_video_get_job_success_and_invalid_json() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path.endswith("/GOOD"):
            return json_response(200, {"job_id": "GOOD", "status": "running"})
        return httpx.Response(200, content=b"not-json")

    client = async_client(handler)
    api = VideoRecognitionClient(client=client)
    assert run(api.get_job("GOOD"))["status"] == "running"
    with pytest.raises(InvalidUpstreamResponseError, match="invalid JSON"):
        run(api.get_job("BAD"))
    run(client.aclose())


def test_video_create_job_uses_confirmed_multipart_contract_and_preserves_id() -> None:
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured.update(multipart_parts(request))
        return json_response(
            200,
            {
                "job_id": "VR_JOB_1",
                "status": "queued",
                "mission_id": "M_RUNTIME",
                "inspection_point_id": "IP_00012",
                "video_id": "VIDEO_1",
            },
        )

    client = async_client(handler)
    api = VideoRecognitionClient(client=client)
    result = run(
        api.create_job(
            video_bytes=b"video-data",
            filename="flight.mp4",
            mission_id="M_RUNTIME",
            inspection_point_id="IP_00012",
            video_id="VIDEO_1",
            max_frames=25,
        )
    )
    assert result["job_id"] == "VR_JOB_1"
    assert captured["video"] == ("flight.mp4", b"video-data", "video/mp4")
    assert captured["mission_id"] == "M_RUNTIME"
    assert captured["inspection_point_id"] == "IP_00012"
    assert captured["video_id"] == "VIDEO_1"
    assert captured["save_annotated_video"] == "true"
    assert captured["save_target_frames"] == "true"
    assert captured["max_frames"] == "25"
    run(client.aclose())


@pytest.mark.parametrize(
    "image_url",
    [
        "http://example.com/frame.jpg",
        "http://127.0.0.1:9000/frame.jpg",
    ],
)
def test_video_media_rejects_other_origin(image_url: str) -> None:
    client = async_client(lambda request: pytest.fail("unsafe URL was requested"))
    api = VideoRecognitionClient(client=client)
    with pytest.raises(UnsafeMediaURLError):
        run(api.download_media(image_url))
    run(client.aclose())


@pytest.mark.parametrize(
    "image_url",
    [
        "/api/v1/video-recognition/media/FRAME.jpg",
        "http://127.0.0.1:8002/api/v1/video-recognition/media/FRAME.jpg",
    ],
)
def test_video_media_accepts_relative_and_configured_absolute_url(image_url: str) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url.host == "127.0.0.1"
        assert request.url.port == 8002
        return httpx.Response(
            200, content=b"jpeg-bytes", headers={"content-type": "image/jpeg"}
        )

    client = async_client(handler)
    api = VideoRecognitionClient(client=client)
    media = run(api.download_media(image_url))
    assert media.content == b"jpeg-bytes"
    assert media.filename == "FRAME.jpg"
    assert media.content_type == "image/jpeg"
    run(client.aclose())


def test_video_media_rejects_redirect() -> None:
    client = async_client(
        lambda request: httpx.Response(
            302, headers={"location": "http://example.com/frame.jpg"}
        )
    )
    api = VideoRecognitionClient(client=client)
    with pytest.raises(UnsafeMediaURLError, match="redirect"):
        run(api.download_media("/frame.jpg"))
    run(client.aclose())


def test_video_media_rejects_oversized_body() -> None:
    client = async_client(
        lambda request: httpx.Response(
            200, content=b"12345", headers={"content-type": "image/jpeg"}
        )
    )
    api = VideoRecognitionClient(client=client, max_download_bytes=4)
    with pytest.raises(MediaTooLargeError):
        run(api.download_media("/frame.jpg"))
    run(client.aclose())


def test_defect_health_success() -> None:
    client = async_client(lambda request: json_response(200, {"status": "ok"}))
    api = DefectDetectionClient(client=client)
    assert run(api.health()) == {"status": "ok"}
    run(client.aclose())


def test_defect_detect_success_multipart_and_opaque_ids() -> None:
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured.update(multipart_parts(request))
        return json_response(
            200,
            {
                "success": True,
                "mission_id": "M_RUNTIME",
                "inspection_point_id": "IP_00012",
                "frame_id": "FRAME:opaque/7",
                "timestamp_ms": 750,
                "image": {"name": "frame.jpg"},
                "detection_count": 0,
                "detections": [],
                "annotated_image_url": "/result.jpg",
            },
        )

    client = async_client(handler)
    api = DefectDetectionClient(client=client)
    result = run(
        api.detect_image(
            image_bytes=b"image-data",
            filename="frame.jpg",
            mission_id="M_RUNTIME",
            inspection_point_id="IP_00012",
            frame_id="FRAME:opaque/7",
            timestamp_ms=750,
        )
    )
    assert result["inspection_point_id"] == "IP_00012"
    assert captured["image"] == ("frame.jpg", b"image-data", "image/jpeg")
    assert captured["mission_id"] == "M_RUNTIME"
    assert captured["inspection_point_id"] == "IP_00012"
    assert captured["frame_id"] == "FRAME:opaque/7"
    assert captured["timestamp_ms"] == "750"
    run(client.aclose())


@pytest.mark.parametrize("status", [400, 500])
def test_defect_detect_http_failure(status: int) -> None:
    client = async_client(lambda request: json_response(status, {"detail": "failed"}))
    api = DefectDetectionClient(client=client)
    with pytest.raises(UpstreamHTTPError, match=f"HTTP {status}"):
        run(
            api.detect_image(
                image_bytes=b"x",
                filename="x.jpg",
                mission_id="M",
                inspection_point_id="IP_00001",
                frame_id="F",
                timestamp_ms=0,
            )
        )
    run(client.aclose())


def test_defect_timeout() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("timeout", request=request)

    client = async_client(handler)
    api = DefectDetectionClient(client=client)
    with pytest.raises(UpstreamTimeoutError):
        run(api.health())
    run(client.aclose())


def test_defect_invalid_json() -> None:
    client = async_client(lambda request: httpx.Response(200, content=b"not-json"))
    api = DefectDetectionClient(client=client)
    with pytest.raises(InvalidUpstreamResponseError, match="invalid JSON"):
        run(api.health())
    run(client.aclose())


@pytest.mark.parametrize(
    "changed_field, changed_value",
    [
        ("mission_id", "OTHER"),
        ("inspection_point_id", "IP_0001"),
        ("frame_id", "REWRITTEN"),
        ("timestamp_ms", 751),
    ],
)
def test_defect_detect_rejects_business_identity_mismatch(
    changed_field: str, changed_value: Any
) -> None:
    payload = {
        "success": True,
        "mission_id": "M",
        "inspection_point_id": "IP_00012",
        "frame_id": "FRAME_X",
        "timestamp_ms": 750,
    }
    payload[changed_field] = changed_value
    client = async_client(lambda request: json_response(200, payload))
    api = DefectDetectionClient(client=client)
    with pytest.raises(IdentityMismatchError, match=changed_field):
        run(
            api.detect_image(
                image_bytes=b"x",
                filename="x.jpg",
                mission_id="M",
                inspection_point_id="IP_00012",
                frame_id="FRAME_X",
                timestamp_ms=750,
            )
        )
    run(client.aclose())
