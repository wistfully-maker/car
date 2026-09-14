"""Safe subprocess execution and idempotency for the existing TTS script."""

import math
import subprocess


def _error(message):
    return {"status": "error", "message": message}


def run_tts(text, command, timeout, runner=None):
    """Execute one TTS request without a shell."""
    if not isinstance(text, str) or not text.strip():
        return _error("text must be non-empty")
    if (
        not isinstance(command, (list, tuple))
        or not command
        or any(not isinstance(value, str) or not value for value in command)
    ):
        return _error("command must be a non-empty argument list")
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or not math.isfinite(timeout)
        or timeout <= 0
    ):
        return _error("timeout must be a positive finite number")

    execute = runner or subprocess.run
    arguments = list(command) + [text]
    try:
        result = execute(
            arguments,
            timeout=float(timeout),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except subprocess.TimeoutExpired:
        return _error("timeout after %ss" % timeout)
    except Exception as exc:
        return _error("TTS process failed: %s" % exc)

    if result.returncode == 0:
        return {"status": "success", "message": ""}
    stderr = result.stderr
    if isinstance(stderr, bytes):
        stderr = stderr.decode("utf-8", errors="replace")
    detail = stderr.strip() if isinstance(stderr, str) else ""
    message = "exit code %d" % result.returncode
    if detail:
        message = "%s: %s" % (message, detail)
    return _error(message)


class TtsBridgeLogic:
    """Run each `(task_id, speech_id)` once and cache its done result."""

    def __init__(self, command, timeout, runner=run_tts):
        self._command = list(command)
        self._timeout = timeout
        self._runner = runner
        self._results = {}

    def handle(self, request):
        key = (request["task_id"], request["speech_id"])
        if key in self._results:
            return dict(self._results[key])

        result = self._runner(
            request["text"],
            self._command,
            self._timeout,
        )
        status = result.get("status")
        message = result.get("message")
        if status not in ("success", "error") or not isinstance(message, str):
            status = "error"
            message = "invalid TTS runner result"
        payload = {
            "protocol_version": 1,
            "task_id": request["task_id"],
            "speech_id": request["speech_id"],
            "status": status,
            "message": message,
        }
        self._results[key] = dict(payload)
        return payload
