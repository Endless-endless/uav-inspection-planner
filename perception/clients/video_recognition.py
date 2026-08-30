"""Async HTTP client for the video-recognition service on port 8002."""

from __future__ import annotations

import mimetypes
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Any
from urllib.parse import unquote

import httpx

from config.perception import (
    HTTP_TIMEOUT_SECONDS,
    MAX_FRAME_DOWNLOAD_BYTES,
    VIDEO_RECOGNITION_BASE_URL,
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


_ALLOWED_IMAGE_CONTENT_TYPES = {
    "image/bmp",
    "image/jpeg",
    "image/png",
    "image/webp",
}


@dataclass(frozen=True)
class DownloadedMedia:
    content: bytes
    filename: str
    content_type: str


class VideoRecognitionClient:
    """Typed boundary around the documented 8002 HTTP contract."""

    def __init__(
        self,
        *,
        base_url: str = VIDEO_RECOGNITION_BASE_URL,
        timeout_seconds: float = HTTP_TIMEOUT_SECONDS,
        max_download_bytes: int = MAX_FRAME_DOWNLOAD_BYTES,
        client: httpx.AsyncClient | None = None,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._origin = self._origin_tuple(httpx.URL(self._base_url))
        self._max_download_bytes = max_download_bytes
        self._owns_client = client is None
        self._client = client or httpx.AsyncClient(
            timeout=httpx.Timeout(timeout_seconds), follow_redirects=False
        )

    async def __aenter__(self) -> "VideoRecognitionClient":
        return self

    async def __aexit__(self, *args: object) -> None:
        await self.aclose()

    async def aclose(self) -> None:
        if self._owns_client:
            await self._client.aclose()

    async def health(self) -> dict[str, Any]:
        return await self._get_json("/api/v1/health")

    async def create_job(
        self,
        *,
        video_bytes: bytes,
        filename: str,
        mission_id: str,
        inspection_point_id: str,
        video_id: str | None = None,
        save_annotated_video: bool = True,
        save_target_frames: bool = True,
        max_frames: int | None = None,
        content_type: str | None = None,
    ) -> dict[str, Any]:
        """Create an asynchronous video job using the documented multipart fields."""
        safe_filename = self._safe_filename(filename, "video.mp4")
        detected_type = content_type or mimetypes.guess_type(safe_filename)[0]
        files = {
            "video": (
                safe_filename,
                video_bytes,
                detected_type or "application/octet-stream",
            )
        }
        data: dict[str, str] = {
            "mission_id": mission_id,
            "inspection_point_id": inspection_point_id,
            "save_annotated_video": str(save_annotated_video).lower(),
            "save_target_frames": str(save_target_frames).lower(),
        }
        if video_id is not None:
            data["video_id"] = video_id
        if max_frames is not None:
            if isinstance(max_frames, bool) or max_frames <= 0:
                raise ValueError("max_frames must be a positive integer")
            data["max_frames"] = str(max_frames)

        payload = await self._request_json(
            "POST", "/api/v1/video-recognition/jobs", files=files, data=data
        )
        self._require_identity(payload, "mission_id", mission_id)
        self._require_identity(
            payload, "inspection_point_id", inspection_point_id
        )
        if video_id is not None:
            self._require_identity(payload, "video_id", video_id)
        return payload

    async def get_job(self, job_id: str) -> dict[str, Any]:
        encoded_job_id = self._encode_path_component(job_id)
        return await self._get_json(
            f"/api/v1/video-recognition/jobs/{encoded_job_id}"
        )

    async def download_media(self, image_url: str) -> DownloadedMedia:
        url = self._resolve_safe_media_url(image_url)
        try:
            async with self._client.stream(
                "GET", url, follow_redirects=False
            ) as response:
                if response.is_redirect:
                    raise UnsafeMediaURLError("media redirects are not allowed")
                self._raise_for_status(response)
                content_type = response.headers.get("content-type", "").split(
                    ";", 1
                )[0].strip().lower()
                if content_type not in _ALLOWED_IMAGE_CONTENT_TYPES:
                    raise InvalidUpstreamResponseError(
                        f"video-recognition media has unsupported Content-Type: "
                        f"{content_type or '<missing>'}"
                    )
                declared_size = response.headers.get("content-length")
                if declared_size is not None:
                    try:
                        if int(declared_size) > self._max_download_bytes:
                            raise MediaTooLargeError(
                                "video-recognition media exceeds configured byte limit"
                            )
                    except ValueError as exc:
                        raise InvalidUpstreamResponseError(
                            "video-recognition returned invalid Content-Length"
                        ) from exc

                chunks: list[bytes] = []
                size = 0
                async for chunk in response.aiter_bytes():
                    size += len(chunk)
                    if size > self._max_download_bytes:
                        raise MediaTooLargeError(
                            "video-recognition media exceeds configured byte limit"
                        )
                    chunks.append(chunk)
        except (UnsafeMediaURLError, InvalidUpstreamResponseError, MediaTooLargeError):
            raise
        except httpx.TimeoutException as exc:
            raise UpstreamTimeoutError(
                "video-recognition media download timed out"
            ) from exc
        except httpx.RequestError as exc:
            raise UpstreamConnectionError(
                "video-recognition media download failed"
            ) from exc

        filename = self._safe_filename(
            unquote(PurePosixPath(url.path).name), "target-frame.jpg"
        )
        return DownloadedMedia(
            content=b"".join(chunks),
            filename=filename,
            content_type=content_type,
        )

    async def _get_json(self, path: str) -> dict[str, Any]:
        return await self._request_json("GET", path)

    async def _request_json(
        self, method: str, path: str, **kwargs: Any
    ) -> dict[str, Any]:
        try:
            response = await self._client.request(
                method, f"{self._base_url}{path}", **kwargs
            )
            self._raise_for_status(response)
        except UpstreamHTTPError:
            raise
        except httpx.TimeoutException as exc:
            raise UpstreamTimeoutError(
                "video-recognition request timed out"
            ) from exc
        except httpx.RequestError as exc:
            raise UpstreamConnectionError(
                "video-recognition request failed"
            ) from exc
        try:
            payload = response.json()
        except ValueError as exc:
            raise InvalidUpstreamResponseError(
                "video-recognition returned invalid JSON"
            ) from exc
        if not isinstance(payload, dict):
            raise InvalidUpstreamResponseError(
                "video-recognition response must be a JSON object"
            )
        return payload

    @staticmethod
    def _origin_tuple(url: httpx.URL) -> tuple[str, str, int | None]:
        if url.scheme not in {"http", "https"} or not url.host:
            raise ValueError("video-recognition base URL must be HTTP(S)")
        return url.scheme, url.host, url.port

    def _resolve_safe_media_url(self, image_url: str) -> httpx.URL:
        try:
            candidate = httpx.URL(image_url)
            if not candidate.is_absolute_url:
                candidate = httpx.URL(f"{self._base_url}/").join(candidate)
        except Exception as exc:
            raise UnsafeMediaURLError("invalid video-recognition media URL") from exc
        if candidate.userinfo or candidate.fragment:
            raise UnsafeMediaURLError(
                "media URL credentials and fragments are not allowed"
            )
        if self._origin_tuple(candidate) != self._origin:
            raise UnsafeMediaURLError(
                "media URL origin differs from configured video service"
            )
        return candidate

    @staticmethod
    def _safe_filename(filename: str, fallback: str) -> str:
        value = PurePosixPath(filename.replace("\\", "/")).name
        return value or fallback

    @staticmethod
    def _encode_path_component(value: str) -> str:
        from urllib.parse import quote

        if not value:
            raise ValueError("job_id must not be empty")
        return quote(value, safe="")

    @staticmethod
    def _require_identity(
        payload: dict[str, Any], field: str, expected: str
    ) -> None:
        if payload.get(field) != expected:
            raise IdentityMismatchError(
                f"video-recognition {field} mismatch: expected {expected!r}, "
                f"got {payload.get(field)!r}"
            )

    @staticmethod
    def _raise_for_status(response: httpx.Response) -> None:
        if response.is_success:
            return
        detail = response.text[:500]
        raise UpstreamHTTPError(
            "video-recognition", response.status_code, detail
        )


__all__ = ["DownloadedMedia", "VideoRecognitionClient"]
