"""Async HTTP client for the defect-detection service on port 8003."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Any

import httpx

from config.perception import (
    DEFECT_DETECTION_BASE_URL,
    HTTP_TIMEOUT_SECONDS,
    MAX_FRAME_DOWNLOAD_BYTES,
)
from perception.clients import (
    IdentityMismatchError,
    InvalidUpstreamResponseError,
    MediaTooLargeError,
    UnsafeMediaURLError,
    UpstreamConnectionError,
    UpstreamHTTPError,
    UpstreamTimeoutError,
)


_ALLOWED_IMAGE_CONTENT_TYPES = {"image/bmp", "image/jpeg", "image/png"}
_SAFE_RESULT_RESOURCE = re.compile(
    r"^[A-Za-z0-9][A-Za-z0-9._-]{0,254}\.(?:jpe?g|png|bmp)$", re.IGNORECASE
)


@dataclass(frozen=True)
class DownloadedDefectMedia:
    content: bytes
    filename: str
    content_type: str


class DefectDetectionClient:
    """Typed boundary around the documented 8003 HTTP contract."""

    def __init__(
        self,
        *,
        base_url: str = DEFECT_DETECTION_BASE_URL,
        timeout_seconds: float = HTTP_TIMEOUT_SECONDS,
        max_download_bytes: int = MAX_FRAME_DOWNLOAD_BYTES,
        client: httpx.AsyncClient | None = None,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._max_download_bytes = max_download_bytes
        self._owns_client = client is None
        self._client = client or httpx.AsyncClient(
            timeout=httpx.Timeout(timeout_seconds), follow_redirects=False
        )

    async def __aenter__(self) -> "DefectDetectionClient":
        return self

    async def __aexit__(self, *args: object) -> None:
        await self.aclose()

    async def aclose(self) -> None:
        if self._owns_client:
            await self._client.aclose()

    async def health(self) -> dict[str, Any]:
        return await self._request_json("GET", "/health")

    async def detect_image(
        self,
        *,
        image_bytes: bytes,
        filename: str,
        mission_id: str,
        inspection_point_id: str,
        frame_id: str,
        timestamp_ms: int,
        content_type: str = "image/jpeg",
    ) -> dict[str, Any]:
        if isinstance(timestamp_ms, bool) or not isinstance(timestamp_ms, int):
            raise TypeError("timestamp_ms must be an integer")
        if timestamp_ms < 0:
            raise ValueError("timestamp_ms must be non-negative")
        safe_filename = PurePosixPath(filename.replace("\\", "/")).name
        if not safe_filename:
            safe_filename = "target-frame.jpg"
        payload = await self._request_json(
            "POST",
            "/api/defect-detection",
            files={"image": (safe_filename, image_bytes, content_type)},
            data={
                "mission_id": mission_id,
                "inspection_point_id": inspection_point_id,
                "frame_id": frame_id,
                "timestamp_ms": str(timestamp_ms),
            },
        )
        if payload.get("success") is not True:
            raise InvalidUpstreamResponseError(
                "defect-detection response success must be true"
            )
        expected = {
            "mission_id": mission_id,
            "inspection_point_id": inspection_point_id,
            "frame_id": frame_id,
            "timestamp_ms": timestamp_ms,
        }
        for field, value in expected.items():
            if payload.get(field) != value:
                raise IdentityMismatchError(
                    f"defect-detection {field} mismatch: expected {value!r}, "
                    f"got {payload.get(field)!r}"
                )
        return payload

    async def download_result_media(
        self, resource_id: str
    ) -> DownloadedDefectMedia:
        """Download one annotated image from the fixed 8003 results directory."""
        if not _SAFE_RESULT_RESOURCE.fullmatch(resource_id):
            raise UnsafeMediaURLError("invalid defect media resource identifier")
        url = f"{self._base_url}/results/{resource_id}"
        try:
            async with self._client.stream(
                "GET", url, follow_redirects=False
            ) as response:
                if response.is_redirect:
                    raise UnsafeMediaURLError("defect media redirects are not allowed")
                if not response.is_success:
                    raise UpstreamHTTPError(
                        "defect-detection", response.status_code
                    )
                content_type = response.headers.get("content-type", "").split(
                    ";", 1
                )[0].strip().lower()
                if content_type not in _ALLOWED_IMAGE_CONTENT_TYPES:
                    raise InvalidUpstreamResponseError(
                        "defect media has unsupported Content-Type"
                    )
                declared_size = response.headers.get("content-length")
                if declared_size is not None:
                    try:
                        if int(declared_size) > self._max_download_bytes:
                            raise MediaTooLargeError(
                                "defect media exceeds configured byte limit"
                            )
                    except ValueError as exc:
                        raise InvalidUpstreamResponseError(
                            "defect media returned invalid Content-Length"
                        ) from exc

                chunks: list[bytes] = []
                size = 0
                async for chunk in response.aiter_bytes():
                    size += len(chunk)
                    if size > self._max_download_bytes:
                        raise MediaTooLargeError(
                            "defect media exceeds configured byte limit"
                        )
                    chunks.append(chunk)
        except (
            InvalidUpstreamResponseError,
            MediaTooLargeError,
            UnsafeMediaURLError,
            UpstreamHTTPError,
        ):
            raise
        except httpx.TimeoutException as exc:
            raise UpstreamTimeoutError("defect media download timed out") from exc
        except httpx.RequestError as exc:
            raise UpstreamConnectionError("defect media download failed") from exc

        return DownloadedDefectMedia(
            content=b"".join(chunks),
            filename=resource_id,
            content_type=content_type,
        )

    async def _request_json(
        self, method: str, path: str, **kwargs: Any
    ) -> dict[str, Any]:
        try:
            response = await self._client.request(
                method, f"{self._base_url}{path}", **kwargs
            )
            if not response.is_success:
                raise UpstreamHTTPError(
                    "defect-detection", response.status_code, response.text[:500]
                )
        except UpstreamHTTPError:
            raise
        except httpx.TimeoutException as exc:
            raise UpstreamTimeoutError("defect-detection request timed out") from exc
        except httpx.RequestError as exc:
            raise UpstreamConnectionError("defect-detection request failed") from exc
        try:
            payload = response.json()
        except ValueError as exc:
            raise InvalidUpstreamResponseError(
                "defect-detection returned invalid JSON"
            ) from exc
        if not isinstance(payload, dict):
            raise InvalidUpstreamResponseError(
                "defect-detection response must be a JSON object"
            )
        return payload


__all__ = ["DefectDetectionClient", "DownloadedDefectMedia"]
