"""JSON protocol shared with the task orchestrator."""

from dataclasses import dataclass
import json


@dataclass(frozen=True)
class TaskIdentity:
    protocol_version: int
    task_id: str
    goal_id: str


def parse_pickup_goal(text):
    try:
        payload = json.loads(text)
    except (TypeError, ValueError) as error:
        raise ValueError("invalid JSON: {}".format(error))
    if payload.get("protocol_version") != 1:
        raise ValueError("protocol_version must be 1")
    task_id = payload.get("task_id")
    goal_id = payload.get("goal_id")
    if not isinstance(task_id, str) or not task_id.strip():
        raise ValueError("task_id must be a non-empty string")
    if not isinstance(goal_id, str) or not goal_id.strip():
        raise ValueError("goal_id must be a non-empty string")
    return TaskIdentity(1, task_id.strip(), goal_id.strip())


def make_pickup_result(identity, status, message):
    if status not in ("arrived", "failed", "cancelled"):
        raise ValueError("unsupported status: {}".format(status))
    if status != "arrived" and not str(message).strip():
        raise ValueError("message is required for non-arrival results")
    payload = {
        "protocol_version": identity.protocol_version,
        "task_id": identity.task_id,
        "goal_id": identity.goal_id,
        "status": status,
        "message": str(message),
    }
    return json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )
