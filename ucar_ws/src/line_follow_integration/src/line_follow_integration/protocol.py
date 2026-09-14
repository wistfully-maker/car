import json
import math


class ProtocolError(ValueError):
    pass


PROTOCOL_VERSION = 1
LINE_STATUSES = frozenset(("waiting_signal", "direction_selected", "following", "success", "failure"))
DIRECTIONS = frozenset(("left_turn", "right_turn", "straight"))


def _load_object(raw):
    try:
        value = json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, ValueError) as exc:
        raise ProtocolError("invalid JSON: %s" % exc)
    if not isinstance(value, dict):
        raise ProtocolError("message must be an object")
    return dict(value)


def _text(value, field):
    if not isinstance(value, str) or not value or value != value.strip():
        raise ProtocolError("%s must be non-blank text without surrounding whitespace" % field)
    return value


def _finite(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("%s must be finite" % field)
    try:
        number = float(value)
    except (OverflowError, TypeError, ValueError):
        raise ProtocolError("%s must be finite" % field)
    if not math.isfinite(number):
        raise ProtocolError("%s must be finite" % field)
    return number


def parse_identity_json(raw):
    value = _load_object(raw)
    if value.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")
    value["task_id"] = _text(value.get("task_id"), "task_id")
    value["goal_id"] = _text(value.get("goal_id"), "goal_id")
    return value


def parse_cancel(raw):
    value = _load_object(raw)
    if value.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")
    value["task_id"] = _text(value.get("task_id"), "task_id")
    value["reason"] = _text(value.get("reason"), "reason")
    return value


def parse_navigation_goal(raw):
    value = parse_identity_json(raw)
    pose = value.get("pose")
    if not isinstance(pose, dict):
        raise ProtocolError("pose must be an object")
    normalized = {"frame_id": _text(pose.get("frame_id"), "frame_id")}
    for field in ("x", "y", "qz", "qw"):
        normalized[field] = _finite(pose.get(field), field)
    if math.hypot(normalized["qz"], normalized["qw"]) < 1e-9:
        raise ProtocolError("orientation must be non-zero")
    value["pose"] = normalized
    return value


def build_arrival(task_id, goal_id, succeeded, message=""):
    task_id = _text(task_id, "task_id")
    goal_id = _text(goal_id, "goal_id")
    if not isinstance(succeeded, bool):
        raise ProtocolError("succeeded must be boolean")
    if not isinstance(message, str):
        raise ProtocolError("message must be text")
    message = message.strip()
    if not succeeded and not message:
        raise ProtocolError("failed arrival requires message")
    return {
        "protocol_version": 1,
        "task_id": task_id,
        "goal_id": goal_id,
        "status": "arrived" if succeeded else "failed",
        "message": message,
    }


def build_line_status(task_id, goal_id, status, direction=None, reason=""):
    task_id = _text(task_id, "task_id")
    goal_id = _text(goal_id, "goal_id")
    if status not in LINE_STATUSES:
        raise ProtocolError("unknown line status")
    if direction is not None and direction not in DIRECTIONS:
        raise ProtocolError("unknown direction")
    if status in ("direction_selected", "following", "success") and direction is None:
        raise ProtocolError("status requires direction")
    if status == "failure" and not isinstance(reason, str):
        raise ProtocolError("failure reason must be text")
    clean_reason = reason.strip() if isinstance(reason, str) else ""
    if status == "failure" and not clean_reason:
        raise ProtocolError("failure requires reason")
    if status != "failure" and clean_reason:
        raise ProtocolError("non-failure status cannot have reason")
    payload = {
        "protocol_version": 1,
        "task_id": task_id,
        "goal_id": goal_id,
        "status": status,
    }
    if direction is not None:
        payload["direction"] = direction
    if clean_reason:
        payload["reason"] = clean_reason
    return payload
