"""Validation and serialization for QR item search protocol version 1."""

import json
import math
from dataclasses import dataclass


PROTOCOL_VERSION = 1
EXPECTED_QR_COUNT = 3


class ProtocolError(ValueError):
    """Raised when a message does not conform to protocol version 1."""


@dataclass(frozen=True)
class StartRequest:
    task_id: str
    search_id: str
    expected_count: int


@dataclass(frozen=True)
class StopRequest:
    task_id: str
    search_id: str
    reason: str


def _require_text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("%s must be a non-empty string" % name)
    return value.strip()


def _require_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("%s must be a finite number" % name)
    if isinstance(value, float) and not math.isfinite(value):
        raise ProtocolError("%s must be a finite number" % name)
    return value


def _parse_object(raw):
    try:
        payload = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise ProtocolError("request must be a JSON object") from error
    if not isinstance(payload, dict):
        raise ProtocolError("request must be a JSON object")
    return payload


def _require_version(payload):
    version = payload.get("protocol_version")
    if type(version) is not int or version != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")


def parse_start_request(raw):
    """Parse and validate a protocol-v1 start request JSON document."""
    payload = _parse_object(raw)
    _require_version(payload)
    expected_count = payload.get("expected_count")
    if type(expected_count) is not int or expected_count != EXPECTED_QR_COUNT:
        raise ProtocolError("expected_count must be %d" % EXPECTED_QR_COUNT)
    return StartRequest(
        task_id=_require_text(payload.get("task_id"), "task_id"),
        search_id=_require_text(payload.get("search_id"), "search_id"),
        expected_count=expected_count,
    )


def parse_stop_request(raw):
    """Parse and validate a protocol-v1 stop request JSON document."""
    payload = _parse_object(raw)
    _require_version(payload)
    return StopRequest(
        task_id=_require_text(payload.get("task_id"), "task_id"),
        search_id=_require_text(payload.get("search_id"), "search_id"),
        reason=_require_text(payload.get("reason"), "reason"),
    )


def _normalize_items(items):
    try:
        values = list(items)
    except TypeError as error:
        raise ProtocolError("items must be iterable") from error

    normalized = []
    for expected_order, item in enumerate(values, start=1):
        if not isinstance(item, dict):
            raise ProtocolError("each item must be an object")
        if type(item.get("order")) is not int or item["order"] != expected_order:
            raise ProtocolError("item order must start at 1 and be consecutive")
        normalized.append({
            "order": item["order"],
            "item_name": _require_text(item.get("item_name"), "item_name"),
            "url": _require_text(item.get("url"), "url"),
            "detected_yaw": _require_number(item.get("detected_yaw"), "detected_yaw"),
        })
    return normalized


def build_search_result(task_id, search_id, stamp, status, items, message):
    """Validate fields and return a normalized protocol-v1 search result."""
    normalized_items = _normalize_items(items)
    if status not in ("searching", "complete", "not_found", "error", "stopped"):
        raise ProtocolError("invalid search status")
    if len(normalized_items) > EXPECTED_QR_COUNT:
        raise ProtocolError("result cannot contain more than %d items" % EXPECTED_QR_COUNT)
    if status in ("searching", "not_found") and len(normalized_items) >= EXPECTED_QR_COUNT:
        raise ProtocolError("%s result must contain fewer than %d items" % (
            status, EXPECTED_QR_COUNT
        ))
    normalized_message = message.strip() if isinstance(message, str) else None
    if normalized_message is None:
        raise ProtocolError("message must be a string")
    if status in ("not_found", "error", "stopped") and not normalized_message:
        raise ProtocolError("terminal status requires a message")
    if status == "complete":
        if len(normalized_items) != EXPECTED_QR_COUNT:
            raise ProtocolError("complete result must contain %d items" % EXPECTED_QR_COUNT)
        if len({item["url"] for item in normalized_items}) != EXPECTED_QR_COUNT:
            raise ProtocolError("complete result URLs must be distinct")
    return {
        "protocol_version": PROTOCOL_VERSION,
        "task_id": _require_text(task_id, "task_id"),
        "search_id": _require_text(search_id, "search_id"),
        "stamp": _require_number(stamp, "stamp"),
        "status": status,
        "items": normalized_items,
        "message": normalized_message,
    }
