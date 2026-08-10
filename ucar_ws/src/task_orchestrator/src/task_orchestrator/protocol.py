import json
import math

from task_orchestrator.categories import category_config


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


def _require_identity(message, field, expected):
    raw_actual = message.get(field)
    actual = require_text(raw_actual, field)
    if raw_actual != actual:
        raise ProtocolError("%s must not contain surrounding whitespace" % field)
    expected_value = require_text(expected, "expected_%s" % field)
    if actual != expected_value:
        raise ProtocolError(
            "%s mismatch: expected %s, got %s"
            % (field, expected_value, actual)
        )
    return actual


def _require_category(value, field):
    category = require_text(value, field)
    try:
        category_config(category)
    except ValueError:
        raise ProtocolError("%s is unsupported: %s" % (field, category))
    return category


def _optional_message(message):
    value = message.get("message", "")
    if not isinstance(value, str):
        raise ProtocolError("message must be text")
    return value.strip()


def _require_failure_message(message):
    return require_text(message.get("message"), "message")


def _require_number(value, field):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ProtocolError("%s must be finite" % field)
    return float(value)


def parse_task_request(raw_json):
    message = load_object(raw_json)
    message["task_id"] = require_text(message.get("task_id"), "task_id")
    message["physical_target_category"] = _require_category(
        message.get("physical_target_category"),
        "physical_target_category",
    )
    message["simulation_target_category"] = _require_category(
        message.get("simulation_target_category"),
        "simulation_target_category",
    )
    message["raw_text"] = require_text(message.get("raw_text"), "raw_text")
    return message


def parse_dependencies_ready(raw_json, expected_task_id):
    message = load_object(raw_json)
    _require_identity(message, "task_id", expected_task_id)
    status = require_text(message.get("status"), "status")
    if status != "ready":
        raise ProtocolError("dependency status must be ready")
    return message


def parse_arrival(raw_json, expected_task_id, expected_goal_id):
    message = load_object(raw_json)
    _require_identity(message, "task_id", expected_task_id)
    _require_identity(message, "goal_id", expected_goal_id)
    status = require_text(message.get("status"), "status")
    if status not in ("arrived", "failed"):
        raise ProtocolError("arrival status must be arrived or failed")
    if status == "failed":
        message["message"] = _require_failure_message(message)
    else:
        message["message"] = _optional_message(message)
    return message


def parse_qr_result(raw_json, expected_task_id, expected_search_id):
    message = load_object(raw_json)
    _require_identity(message, "task_id", expected_task_id)
    _require_identity(message, "search_id", expected_search_id)
    status = require_text(message.get("status"), "status")
    allowed_statuses = ("searching", "complete", "not_found", "error", "stopped")
    if status not in allowed_statuses:
        raise ProtocolError("unsupported QR status: %s" % status)

    message["stamp"] = _require_number(message.get("stamp"), "stamp")
    items = message.get("items", [])
    if not isinstance(items, list):
        raise ProtocolError("items must be a list")
    message["items"] = _parse_qr_items(items)
    if status == "complete":
        if len(message["items"]) != 3:
            raise ProtocolError(
                "complete QR result must contain exactly three items"
            )
        names = {item["item_name"] for item in message["items"]}
        urls = {item["url"] for item in message["items"]}
        if len(names) != 3 or len(urls) != 3:
            raise ProtocolError("QR names and URLs must be distinct")
        message["message"] = _optional_message(message)
    else:
        if status in ("searching", "not_found") and len(message["items"]) >= 3:
            raise ProtocolError("%s QR result must be partial" % status)
        if status in ("not_found", "error", "stopped"):
            message["message"] = _require_failure_message(message)
        else:
            message["message"] = _optional_message(message)
    return message


def _parse_qr_items(items):
    if len(items) > 3:
        raise ProtocolError("QR result cannot contain more than three items")
    parsed = []
    for expected_order, item in enumerate(items, start=1):
        index = expected_order - 1
        if not isinstance(item, dict):
            raise ProtocolError("items[%d] must be an object" % index)
        order = item.get("order")
        if (
            isinstance(order, bool)
            or not isinstance(order, int)
            or order != expected_order
        ):
            raise ProtocolError("QR orders must start at 1 and be consecutive")
        name = require_text(item.get("item_name"), "items[%d].item_name" % index)
        url = require_text(item.get("url"), "items[%d].url" % index)
        yaw = _require_number(
            item.get("detected_yaw"),
            "items[%d].detected_yaw" % index,
        )
        parsed_item = dict(item)
        parsed_item["item_name"] = name
        parsed_item["url"] = url
        parsed_item["detected_yaw"] = yaw
        parsed.append(parsed_item)
    return parsed


def parse_llm_result(raw_json, context):
    message = load_object(raw_json)
    if not isinstance(context, dict):
        raise ProtocolError("LLM context must be an object")
    _require_identity(message, "task_id", context.get("task_id"))
    _require_identity(message, "request_id", context.get("request_id"))
    status = require_text(message.get("status"), "status")
    if status not in ("success", "error"):
        raise ProtocolError("LLM status must be success or error")
    if status == "error":
        message["message"] = _require_failure_message(message)
        return message

    candidates = _candidate_map(context.get("candidates"))
    physical_category = _require_category(
        context.get("physical_target_category"),
        "physical_target_category",
    )
    simulation_category = _require_category(
        context.get("simulation_target_category"),
        "simulation_target_category",
    )
    message["physical"] = _parse_llm_selection(
        message.get("physical"),
        "physical",
        physical_category,
        candidates,
    )
    message["simulation"] = _parse_llm_selection(
        message.get("simulation"),
        "simulation",
        simulation_category,
        candidates,
    )
    message["message"] = _optional_message(message)
    return message


def parse_speak_request(raw_json):
    message = load_object(raw_json)
    message["task_id"] = require_text(message.get("task_id"), "task_id")
    message["speech_id"] = require_text(
        message.get("speech_id"),
        "speech_id",
    )
    message["text"] = require_text(message.get("text"), "text")
    return message


def _candidate_map(candidates):
    if not isinstance(candidates, list) or not candidates:
        raise ProtocolError("candidates must be a non-empty list")
    result = {}
    for index, candidate in enumerate(candidates):
        if not isinstance(candidate, dict):
            raise ProtocolError("candidates[%d] must be an object" % index)
        order = candidate.get("order")
        if isinstance(order, bool) or not isinstance(order, int):
            raise ProtocolError("candidates[%d].order must be an integer" % index)
        if order in result:
            raise ProtocolError("candidate orders must be distinct")
        result[order] = require_text(
            candidate.get("item_name"),
            "candidates[%d].item_name" % index,
        )
    return result


def _parse_llm_selection(selection, field, expected_category, candidates):
    if not isinstance(selection, dict):
        raise ProtocolError("%s must be an object" % field)
    order = selection.get("selected_order")
    if isinstance(order, bool) or not isinstance(order, int):
        raise ProtocolError("%s.selected_order must be an integer" % field)
    if order not in candidates:
        raise ProtocolError("%s.selected_order is not a candidate" % field)
    item = require_text(selection.get("selected_item"), "%s.selected_item" % field)
    if item != candidates[order]:
        raise ProtocolError("%s.selected_item does not match its order" % field)
    category = _require_category(selection.get("category"), "%s.category" % field)
    if category != expected_category:
        raise ProtocolError("%s.category does not match the target" % field)
    workshop = require_text(selection.get("workshop"), "%s.workshop" % field)
    if workshop != category_config(expected_category)["workshop"]:
        raise ProtocolError("%s.workshop does not match the category" % field)
    result = dict(selection)
    result.update(selected_item=item, category=category, workshop=workshop)
    return result


def parse_speech_done(
    raw_json,
    expected_task_id,
    expected_speech_id,
):
    message = load_object(raw_json)
    _require_identity(message, "task_id", expected_task_id)
    _require_identity(message, "speech_id", expected_speech_id)
    status = require_text(message.get("status"), "status")
    if status not in ("success", "error"):
        raise ProtocolError("speech status must be success or error")
    if status == "error":
        message["message"] = _require_failure_message(message)
    else:
        message["message"] = _optional_message(message)
    return message


def parse_cancel(raw_json, expected_task_id):
    message = load_object(raw_json)
    _require_identity(message, "task_id", expected_task_id)
    raw_reason = message.get("reason")
    message["reason"] = require_text(raw_reason, "reason")
    if raw_reason != message["reason"]:
        raise ProtocolError("reason must not contain surrounding whitespace")
    return message
