"""ROS-independent TCP protocol helpers for the Gazebo soft gate."""

import json

from task_orchestrator.categories import category_config


class BridgeProtocolError(ValueError):
    pass


def _text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise BridgeProtocolError("%s must be non-empty text" % field)
    clean = value.strip()
    if clean != value:
        raise BridgeProtocolError("%s must not contain surrounding whitespace" % field)
    return clean


def _version(message):
    if not isinstance(message, dict):
        raise BridgeProtocolError("message must be an object")
    value = message.get("protocol_version")
    if isinstance(value, bool) or not isinstance(value, int) or value != 1:
        raise BridgeProtocolError("unsupported protocol version")


def validate_start(message):
    _version(message)
    parsed = dict(message)
    for field in ("task_id", "goal_id", "selected_item", "target_category",
                  "target_workshop"):
        parsed[field] = _text(message.get(field), field)
    try:
        expected_workshop = category_config(parsed["target_category"])["workshop"]
    except ValueError as exc:
        raise BridgeProtocolError(str(exc))
    if parsed["target_workshop"] != expected_workshop:
        raise BridgeProtocolError("target_workshop does not match target_category")
    return parsed


def validate_complete(message):
    _version(message)
    parsed = dict(message)
    parsed["task_id"] = _text(message.get("task_id"), "task_id")
    parsed["goal_id"] = _text(message.get("goal_id"), "goal_id")
    parsed["status"] = _text(message.get("status"), "status")
    if parsed["status"] not in ("success", "failure"):
        raise BridgeProtocolError("status must be success or failure")
    if parsed["status"] == "failure":
        parsed["reason"] = _text(message.get("reason"), "reason")
    elif "reason" in message:
        raise BridgeProtocolError("success must not include reason")
    return parsed


def encode_line(message):
    if not isinstance(message, dict):
        raise BridgeProtocolError("message must be an object")
    return (json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


class JsonLineDecoder:
    def __init__(self, max_frame_bytes=8192):
        if isinstance(max_frame_bytes, bool) or int(max_frame_bytes) <= 0:
            raise ValueError("max_frame_bytes must be positive")
        self.max_frame_bytes = int(max_frame_bytes)
        self._buffer = bytearray()

    def feed(self, data):
        if not isinstance(data, (bytes, bytearray)):
            raise BridgeProtocolError("socket data must be bytes")
        self._buffer.extend(data)
        frames = []
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                if len(self._buffer) > self.max_frame_bytes:
                    self._buffer.clear()
                    raise BridgeProtocolError("frame exceeds maximum size")
                return frames
            if newline > self.max_frame_bytes:
                self._buffer.clear()
                raise BridgeProtocolError("frame exceeds maximum size")
            raw = bytes(self._buffer[:newline])
            del self._buffer[:newline + 1]
            if not raw:
                continue
            try:
                message = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, ValueError) as exc:
                raise BridgeProtocolError("invalid JSON frame: %s" % exc)
            if not isinstance(message, dict):
                raise BridgeProtocolError("JSON frame must be an object")
            frames.append(message)


class GazeboBridgeSession:
    def __init__(self, allowed_client_ip=""):
        self.allowed_client_ip = allowed_client_ip.strip()
        self.connected = False
        self.active = None
        self._seen = set()

    def connect(self, peer_ip):
        peer_ip = _text(peer_ip, "peer_ip")
        if self.allowed_client_ip and peer_ip != self.allowed_client_ip:
            return False
        if self.connected:
            return False
        self.connected = True
        return True

    @staticmethod
    def _failure(start, reason):
        return {
            "protocol_version": 1,
            "task_id": start["task_id"],
            "goal_id": start["goal_id"],
            "status": "failure",
            "reason": reason,
        }

    def start(self, message):
        start = validate_start(message)
        identity = (start["task_id"], start["goal_id"])
        if identity in self._seen:
            return None
        self._seen.add(identity)
        if not self.connected:
            return {"complete": self._failure(start, "desktop_not_connected")}
        if self.active is not None:
            return {"complete": self._failure(start, "bridge_busy")}
        self.active = start
        return {"send": start}

    def receive(self, message):
        complete = validate_complete(message)
        if self.active is None:
            return None
        if (complete["task_id"], complete["goal_id"]) != (
            self.active["task_id"], self.active["goal_id"]
        ):
            return None
        self.active = None
        return {"complete": complete}

    def disconnect(self):
        self.connected = False
        if self.active is None:
            return None
        active = self.active
        self.active = None
        return {"complete": self._failure(active, "connection_lost")}
