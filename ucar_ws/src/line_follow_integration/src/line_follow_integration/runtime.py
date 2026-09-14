import math
import os
import subprocess


ROUTE_SCRIPTS = {
    "left_turn": "follow_left_v5.py",
    "right_turn": "follow_right_v5.py",
    "straight": "follow_mid_v4.py",
}


class ImageHealthGate:
    def __init__(self, clock, max_age, recovery_grace):
        self.clock = clock
        self.max_age = float(max_age)
        self.recovery_grace = float(recovery_grace)
        if self.max_age <= 0 or self.recovery_grace <= 0:
            raise ValueError("image timing must be positive")
        self.reset()

    def reset(self):
        self.last_frame = None
        self.unhealthy_since = None
        self.stop_emitted = False

    def observe_frame(self, timestamp):
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("image timestamp must be finite")
        self.last_frame = timestamp
        self.unhealthy_since = None
        self.stop_emitted = False

    def allows_motion(self):
        now = float(self.clock())
        return self.last_frame is not None and 0.0 <= now - self.last_frame <= self.max_age

    def poll(self):
        now = float(self.clock())
        if self.allows_motion():
            self.unhealthy_since = None
            self.stop_emitted = False
            return "healthy"
        if self.unhealthy_since is None:
            self.unhealthy_since = now
        if now - self.unhealthy_since >= self.recovery_grace:
            return "failure"
        if not self.stop_emitted:
            self.stop_emitted = True
            return "stop"
        return "blocked"


def any_process_running(processes):
    for process in processes:
        try:
            if process.poll() is None:
                return True
        except OSError:
            continue
    return False


def spawn_process(popen, command):
    try:
        return (popen(command), "")
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        return (None, str(exc))


class LineFollowSession:
    def __init__(self, result_file, stop_file, clock, direction_timeout, follow_timeout):
        self.result_file = result_file
        self.stop_file = stop_file
        self.clock = clock
        self.direction_timeout = float(direction_timeout)
        self.follow_timeout = float(follow_timeout)
        if self.direction_timeout <= 0 or self.follow_timeout <= 0:
            raise ValueError("timeouts must be positive")
        self.clear()

    def start(self, task_id, goal_id):
        if not isinstance(task_id, str) or not task_id.strip():
            raise ValueError("task_id must be text")
        if not isinstance(goal_id, str) or not goal_id.strip():
            raise ValueError("goal_id must be text")
        for path in (self.result_file, self.stop_file):
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
        self.task_id = task_id
        self.goal_id = goal_id
        self.activation_time = float(self.clock())
        self.follow_started = None
        self.direction = None
        self.terminal = None

    def _fresh_text(self, path):
        try:
            if os.path.getmtime(path) < self.activation_time:
                return None
            with open(path, "r", encoding="utf-8") as handle:
                return handle.read().strip()
        except (FileNotFoundError, OSError, UnicodeError):
            return None

    def poll_direction(self):
        if self.direction is not None:
            return ("direction_selected", self.direction)
        value = self._fresh_text(self.result_file)
        if value in ROUTE_SCRIPTS:
            self.lock_direction(value)
            return ("direction_selected", self.direction)
        if float(self.clock()) - self.activation_time >= self.direction_timeout:
            self.lock_direction("straight")
            return ("direction_selected", self.direction)
        return ("waiting_signal", None)

    def lock_direction(self, direction):
        if direction not in ROUTE_SCRIPTS:
            raise ValueError("unknown direction")
        if self.direction is None:
            self.direction = direction
            self.follow_started = float(self.clock())
        return self.direction

    def poll_follow(self, child_running):
        if self.terminal is not None:
            return self.terminal
        if self.direction is None or self.follow_started is None:
            raise RuntimeError("direction is not locked")
        if self._fresh_text(self.stop_file) is not None:
            self.terminal = ("success", self.direction)
        elif not child_running:
            self.terminal = ("failure", "line follower exited before final stop")
        elif float(self.clock()) - self.follow_started >= self.follow_timeout:
            self.terminal = ("failure", "line follow timed out")
        else:
            return ("following", None)
        return self.terminal

    def fail(self, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be non-blank text")
        self.terminal = ("failure", reason.strip())
        return self.terminal

    def clear(self):
        self.task_id = None
        self.goal_id = None
        self.activation_time = None
        self.follow_started = None
        self.direction = None
        self.terminal = None
