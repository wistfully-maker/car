"""Protocol v1 handling for the dual-stage delivery task (ROS-free).

Parses both navigation goal types and builds both arrival result messages.
All identity fields are preserved exactly as received; surrounding whitespace
is rejected rather than silently stripped.
"""

import json
import math

PHASE_PHYSICAL = "physical"
PHASE_SIMULATION = "simulation"
PHASES = (PHASE_PHYSICAL, PHASE_SIMULATION)

ALLOWED_WORKSHOPS = (
    "食品加工车间",
    "日用品加工车间",
    "电子产品生产车间",
)

PROTOCOL_VERSION = 1


class ProtocolError(ValueError):
    """Raised when a ROS JSON message violates protocol v1."""


def load_object(raw_json):
    try:
        value = json.loads(raw_json)
    except (TypeError, ValueError) as exc:
        raise ProtocolError("message must be valid JSON: %s" % exc)
    if not isinstance(value, dict):
        raise ProtocolError("message must be a JSON object")
    version = value.get("protocol_version")
    if isinstance(version, bool) or not isinstance(version, int) or version != 1:
        raise ProtocolError("unsupported protocol version")
    return value


def require_text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("%s must be non-empty" % field)
    return value.strip()


def _require_strict_text(message, field):
    raw = message.get(field)
    text = require_text(raw, field)
    if raw != text:
        raise ProtocolError("%s must not contain surrounding whitespace" % field)
    return text


def _require_workshop(value):
    workshop = _strict_text(value, "target_workshop")
    if workshop not in ALLOWED_WORKSHOPS:
        raise ProtocolError("unsupported target_workshop: %s" % workshop)
    return workshop


def _strict_text(value, field):
    text = require_text(value, field)
    if value != text:
        raise ProtocolError("%s must not contain surrounding whitespace" % field)
    return text


def _parse_navigation_goal(raw_json):
    message = load_object(raw_json)
    message["task_id"] = _strict_text(message.get("task_id"), "task_id")
    message["goal_id"] = _strict_text(message.get("goal_id"), "goal_id")
    message["target_workshop"] = _require_workshop(message.get("target_workshop"))
    message["selected_item"] = _strict_text(
        message.get("selected_item"), "selected_item"
    )
    return message


def parse_delivery_goal(raw_json):
    """Parse a /task/delivery_navigation_goal message (phase=physical)."""
    goal = _parse_navigation_goal(raw_json)
    goal["phase"] = PHASE_PHYSICAL
    return goal


def parse_simulation_goal(raw_json):
    """Parse a /task/simulation_navigation_goal message (phase=simulation)."""
    goal = _parse_navigation_goal(raw_json)
    goal["phase"] = PHASE_SIMULATION
    return goal


def _optional_message(value):
    if not isinstance(value, str):
        raise ProtocolError("message must be text")
    return value.strip()


def build_arrival(phase, task_id, goal_id, status, message):
    """Build an arrival result dict for physical or simulation delivery.

    status must be "arrived" (message may be empty) or "failed" (message must
    be non-empty and actionable). Identifiers are preserved verbatim.
    """
    if phase not in PHASES:
        raise ProtocolError("unsupported delivery phase: %s" % phase)
    task_id = _strict_text(task_id, "task_id")
    goal_id = _strict_text(goal_id, "goal_id")
    if status not in ("arrived", "failed"):
        raise ProtocolError("arrival status must be arrived or failed")
    if status == "failed":
        message = require_text(message, "message")
    else:
        message = _optional_message(message)
    return {
        "protocol_version": PROTOCOL_VERSION,
        "task_id": task_id,
        "goal_id": goal_id,
        "status": status,
        "message": message,
    }


def _require_finite_number(value, field):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ProtocolError("%s must be finite" % field)
    return float(value)


def parse_safe_stop_ack(raw_json, expected_task_id):
    """Parse a /task/delivery_stop_ack message (reserved for handoff audits)."""
    message = load_object(raw_json)
    task_id = _require_strict_text(message, "task_id")
    if task_id != expected_task_id:
        raise ProtocolError(
            "task_id mismatch: expected %s, got %s" % (expected_task_id, task_id)
        )
    status = require_text(message.get("status"), "status")
    if status != "stopped":
        raise ProtocolError("stop status must be stopped")
    message["status"] = status
    return message
