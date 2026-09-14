"""Pure pass-through waypoint state logic."""

from dataclasses import dataclass
import math


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Waypoint:
    name: str
    kind: str
    x: float
    y: float
    yaw: float
    switch_radius: float = 0.35
    exit_radius: float = 0.45
    heading_tolerance: float = math.radians(40.0)
    minimum_pass_speed: float = 0.08
    position_tolerance: float = 0.15
    yaw_tolerance: float = 0.15
    settle_time: float = 0.5


@dataclass(frozen=True)
class Decision:
    send_next_goal: bool = False
    arrived: bool = False
    publish_stop: bool = False
    error: str = ""


class RouteProgress:
    def __init__(self, waypoints, terminal_speed_tolerance=0.03):
        if not waypoints:
            raise ValueError("at least one waypoint is required")
        if waypoints[-1].kind != "terminal":
            raise ValueError("last waypoint must be terminal")
        self.waypoints = list(waypoints)
        self.terminal_speed_tolerance = float(
            terminal_speed_tolerance
        )
        self.current_index = 0
        self._terminal_stable_since = None

    @property
    def current(self):
        return self.waypoints[self.current_index]

    def update(self, x, y, yaw, linear_speed, now):
        waypoint = self.current
        distance = math.hypot(waypoint.x - x, waypoint.y - y)
        heading_error = abs(normalize_angle(waypoint.yaw - yaw))

        if waypoint.kind == "pass_through":
            if distance > waypoint.switch_radius:
                return Decision()
            self.current_index += 1
            return Decision(send_next_goal=True)

        pose_valid = (
            distance <= waypoint.position_tolerance
            and heading_error <= waypoint.yaw_tolerance
        )
        speed_valid = abs(linear_speed) <= self.terminal_speed_tolerance
        if not pose_valid or not speed_valid:
            self._terminal_stable_since = None
            return Decision()
        if self._terminal_stable_since is None:
            self._terminal_stable_since = float(now)
            return Decision()
        if now - self._terminal_stable_since >= waypoint.settle_time:
            return Decision(arrived=True, publish_stop=True)
        return Decision()

    def advance_stopped_pass_through(self):
        if self.current.kind != "pass_through":
            return Decision()
        self.current_index += 1
        return Decision(send_next_goal=True)
