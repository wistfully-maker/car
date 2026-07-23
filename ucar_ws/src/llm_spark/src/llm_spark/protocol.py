"""Protocol-v1 validation for Spark dual-target classification."""

import json


PROTOCOL_VERSION = 1
CATEGORY_WORKSHOPS = {
    "食品": "食品加工车间",
    "日用品": "日用品加工车间",
    "电子产品": "电子产品生产车间",
}


class ProtocolError(ValueError):
    """Raised when request or model output violates the protocol."""


def _text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("%s must be non-empty text" % field)
    return value.strip()


def _category(value, field):
    value = _text(value, field)
    if value not in CATEGORY_WORKSHOPS:
        raise ProtocolError("%s is not a competition category" % field)
    return value


def parse_request(raw):
    try:
        request = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise ProtocolError("request must be valid JSON") from error
    if not isinstance(request, dict):
        raise ProtocolError("request must be an object")
    if type(request.get("protocol_version")) is not int:
        raise ProtocolError("protocol_version must be an integer")
    if request["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")

    parsed = {
        "protocol_version": PROTOCOL_VERSION,
        "task_id": _text(request.get("task_id"), "task_id"),
        "request_id": _text(request.get("request_id"), "request_id"),
        "physical_target_category": _category(
            request.get("physical_target_category"),
            "physical_target_category",
        ),
        "simulation_target_category": _category(
            request.get("simulation_target_category"),
            "simulation_target_category",
        ),
    }
    candidates = request.get("candidates")
    if not isinstance(candidates, list) or len(candidates) != 3:
        raise ProtocolError("candidates must contain exactly three items")
    parsed_candidates = []
    names = set()
    for expected_order, candidate in enumerate(candidates, start=1):
        if not isinstance(candidate, dict):
            raise ProtocolError("candidate must be an object")
        if candidate.get("order") != expected_order:
            raise ProtocolError("candidate orders must be consecutive")
        name = _text(candidate.get("item_name"), "candidate.item_name")
        if name in names:
            raise ProtocolError("candidate item names must be distinct")
        names.add(name)
        parsed_candidates.append({
            "order": expected_order,
            "item_name": name,
        })
    parsed["candidates"] = parsed_candidates
    return parsed


def build_prompt(request):
    candidate_lines = "\n".join(
        "%d. %s" % (item["order"], item["item_name"])
        for item in request["candidates"]
    )
    return """请从同一候选列表中完成两个独立选择。

实物目标母类：{physical}
仿真目标母类：{simulation}

候选物品：
{candidates}

只能从候选物品中选择。严格返回一个 JSON 对象，不要解释或使用 Markdown：
{{
  "physical": {{
    "selected_order": 1,
    "selected_item": "",
    "category": "",
    "workshop": ""
  }},
  "simulation": {{
    "selected_order": 2,
    "selected_item": "",
    "category": "",
    "workshop": ""
  }}
}}""".format(
        physical=request["physical_target_category"],
        simulation=request["simulation_target_category"],
        candidates=candidate_lines,
    )


def _selection(value, field, expected_category, candidates):
    if not isinstance(value, dict):
        raise ProtocolError("%s must be an object" % field)
    order = value.get("selected_order")
    if isinstance(order, bool) or not isinstance(order, int):
        raise ProtocolError("%s.selected_order must be an integer" % field)
    candidate_map = {item["order"]: item["item_name"] for item in candidates}
    if order not in candidate_map:
        raise ProtocolError("%s selection is not a candidate" % field)
    item = _text(value.get("selected_item"), field + ".selected_item")
    if item != candidate_map[order]:
        raise ProtocolError("%s item does not match selected_order" % field)
    category = _category(value.get("category"), field + ".category")
    if category != expected_category:
        raise ProtocolError("%s category does not match target" % field)
    workshop = _text(value.get("workshop"), field + ".workshop")
    if workshop != CATEGORY_WORKSHOPS[category]:
        raise ProtocolError("%s workshop does not match category" % field)
    return {
        "selected_order": order,
        "selected_item": item,
        "category": category,
        "workshop": workshop,
    }


def build_success_result(request, model_result):
    if not isinstance(model_result, dict):
        raise ProtocolError("model result must be an object")
    return {
        "protocol_version": PROTOCOL_VERSION,
        "task_id": request["task_id"],
        "request_id": request["request_id"],
        "status": "success",
        "physical": _selection(
            model_result.get("physical"),
            "physical",
            request["physical_target_category"],
            request["candidates"],
        ),
        "simulation": _selection(
            model_result.get("simulation"),
            "simulation",
            request["simulation_target_category"],
            request["candidates"],
        ),
        "message": "",
    }


def build_error_result(request, message):
    return {
        "protocol_version": PROTOCOL_VERSION,
        "task_id": _text(request.get("task_id"), "task_id"),
        "request_id": _text(request.get("request_id"), "request_id"),
        "status": "error",
        "message": _text(message, "message"),
    }
