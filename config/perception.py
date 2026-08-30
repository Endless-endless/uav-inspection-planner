"""Configuration for the external perception HTTP services.

Values are process configuration only. API callers cannot override service origins.
"""

from __future__ import annotations

import os


def _positive_float(name: str, default: float) -> float:
    value = float(os.getenv(name, str(default)))
    if value <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return value


def _positive_int(name: str, default: int) -> int:
    value = int(os.getenv(name, str(default)))
    if value <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return value


VIDEO_RECOGNITION_BASE_URL = os.getenv(
    "VIDEO_RECOGNITION_BASE_URL", "http://127.0.0.1:8002"
).rstrip("/")
DEFECT_DETECTION_BASE_URL = os.getenv(
    "DEFECT_DETECTION_BASE_URL", "http://127.0.0.1:8003"
).rstrip("/")

HTTP_TIMEOUT_SECONDS = _positive_float("PERCEPTION_HTTP_TIMEOUT_SECONDS", 30.0)
VIDEO_POLL_INTERVAL_SECONDS = _positive_float(
    "VIDEO_POLL_INTERVAL_SECONDS", 1.0
)
VIDEO_POLL_DEADLINE_SECONDS = _positive_float(
    "VIDEO_POLL_DEADLINE_SECONDS", 600.0
)
MAX_FRAME_DOWNLOAD_BYTES = _positive_int(
    "MAX_FRAME_DOWNLOAD_BYTES", 20 * 1024 * 1024
)


__all__ = [
    "DEFECT_DETECTION_BASE_URL",
    "HTTP_TIMEOUT_SECONDS",
    "MAX_FRAME_DOWNLOAD_BYTES",
    "VIDEO_POLL_DEADLINE_SECONDS",
    "VIDEO_POLL_INTERVAL_SECONDS",
    "VIDEO_RECOGNITION_BASE_URL",
]
