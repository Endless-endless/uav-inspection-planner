"""Shared exceptions for the external perception HTTP clients."""

from __future__ import annotations


class PerceptionClientError(RuntimeError):
    """Base class for failures at the perception HTTP boundary."""


class UpstreamConnectionError(PerceptionClientError):
    """The upstream service could not be reached."""


class UpstreamTimeoutError(PerceptionClientError):
    """The upstream request exceeded its configured timeout."""


class UpstreamHTTPError(PerceptionClientError):
    """The upstream service returned a non-success HTTP status."""

    def __init__(self, service: str, status_code: int, detail: str = "") -> None:
        self.service = service
        self.status_code = status_code
        self.detail = detail
        message = f"{service} returned HTTP {status_code}"
        if detail:
            message += f": {detail}"
        super().__init__(message)


class InvalidUpstreamResponseError(PerceptionClientError):
    """The upstream response did not match its documented contract."""


class IdentityMismatchError(PerceptionClientError):
    """Business identity fields changed across the HTTP boundary."""


class UnsafeMediaURLError(PerceptionClientError):
    """A media URL did not belong to the configured video-service origin."""


class MediaTooLargeError(PerceptionClientError):
    """Downloaded media exceeded the configured in-memory byte limit."""


__all__ = [
    "IdentityMismatchError",
    "InvalidUpstreamResponseError",
    "MediaTooLargeError",
    "PerceptionClientError",
    "UnsafeMediaURLError",
    "UpstreamConnectionError",
    "UpstreamHTTPError",
    "UpstreamTimeoutError",
]
