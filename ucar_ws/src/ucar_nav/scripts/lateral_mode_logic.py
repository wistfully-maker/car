#!/usr/bin/env python3
"""Pure path-geometry logic for switching TEB lateral limits."""

from dataclasses import dataclass
import math


_EPSILON = 1e-6


@dataclass(frozen=True)
class PathMetrics:
    turn_angle_deg: float
    exit_heading_rad: float
    analyzed_length: float


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _deduplicate(points):
    clean = []
    for point in points:
        candidate = (float(point[0]), float(point[1]))
        if not clean or math.hypot(
            candidate[0] - clean[-1][0], candidate[1] - clean[-1][1]
        ) >= _EPSILON:
            clean.append(candidate)
    return clean


def _path_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def _clip_path(points, maximum_length):
    if not points or maximum_length <= 0.0:
        return points[:1]
    clipped = [points[0]]
    travelled = 0.0
    for start, end in zip(points, points[1:]):
        segment = math.hypot(end[0] - start[0], end[1] - start[1])
        if travelled + segment <= maximum_length + _EPSILON:
            clipped.append(end)
            travelled += segment
            continue
        remaining = maximum_length - travelled
        if remaining > _EPSILON and segment > _EPSILON:
            ratio = remaining / segment
            clipped.append(
                (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            )
        break
    return clipped


def resample_path(points, spacing):
    """Return points sampled at approximately equal arc-length intervals."""
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    clean = _deduplicate(points)
    if len(clean) < 2:
        return clean

    total = _path_length(clean)
    distances = []
    current = 0.0
    while current < total:
        distances.append(current)
        current += spacing
    distances.append(total)

    sampled = []
    segment_index = 0
    segment_start_distance = 0.0
    segment_length = math.hypot(
        clean[1][0] - clean[0][0], clean[1][1] - clean[0][1]
    )
    for target in distances:
        while (
            segment_index < len(clean) - 2
            and target > segment_start_distance + segment_length
        ):
            segment_start_distance += segment_length
            segment_index += 1
            segment_length = math.hypot(
                clean[segment_index + 1][0] - clean[segment_index][0],
                clean[segment_index + 1][1] - clean[segment_index][1],
            )
        ratio = (
            0.0
            if segment_length < _EPSILON
            else (target - segment_start_distance) / segment_length
        )
        ratio = min(1.0, max(0.0, ratio))
        start = clean[segment_index]
        end = clean[segment_index + 1]
        sampled.append(
            (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
        )
    return sampled


def _aggregate_heading(points, start_index, end_index):
    dx = 0.0
    dy = 0.0
    for index in range(start_index, end_index):
        dx += points[index + 1][0] - points[index][0]
        dy += points[index + 1][1] - points[index][1]
    return math.atan2(dy, dx)


def analyze_path(points, lookahead_distance, resample_spacing):
    """Measure heading change between the entrance and exit of a path window."""
    clean = _deduplicate(points)
    clipped = _clip_path(clean, float(lookahead_distance))
    analyzed_length = _path_length(clipped)
    sampled = resample_path(clipped, float(resample_spacing))
    if len(sampled) < 3:
        return PathMetrics(0.0, 0.0, analyzed_length)

    segment_count = len(sampled) - 1
    window = max(1, int(math.ceil(segment_count * 0.3)))
    entrance = _aggregate_heading(sampled, 0, window)
    exit_heading = _aggregate_heading(
        sampled, segment_count - window, segment_count
    )
    turn_angle = abs(math.degrees(normalize_angle(exit_heading - entrance)))
    return PathMetrics(turn_angle, exit_heading, analyzed_length)


def mode_parameters(
    mode,
    straight_max_vel_y,
    straight_acc_lim_y,
    corner_max_vel_y,
    corner_acc_lim_y,
):
    if mode == "STRAIGHT":
        return {
            "max_vel_y": straight_max_vel_y,
            "acc_lim_y": straight_acc_lim_y,
        }
    if mode == "CORNER":
        return {
            "max_vel_y": corner_max_vel_y,
            "acc_lim_y": corner_acc_lim_y,
        }
    raise ValueError("unknown lateral mode: {}".format(mode))


class LateralModeStateMachine:
    def __init__(
        self,
        enter_angle_deg,
        exit_angle_deg,
        heading_exit_tolerance_deg,
        exit_hold_time,
    ):
        self.enter_angle_deg = float(enter_angle_deg)
        self.exit_angle_deg = float(exit_angle_deg)
        self.heading_exit_tolerance_deg = float(heading_exit_tolerance_deg)
        self.exit_hold_time = float(exit_hold_time)
        self.mode = "STRAIGHT"
        self._exit_candidate_since = None

    def update(self, now, metrics, robot_heading_rad, plan_valid=True):
        if not plan_valid:
            self.mode = "STRAIGHT"
            self._exit_candidate_since = None
            return self.mode

        if self.mode == "STRAIGHT":
            if metrics.turn_angle_deg >= self.enter_angle_deg:
                self.mode = "CORNER"
            return self.mode

        heading_error_deg = abs(
            math.degrees(
                normalize_angle(robot_heading_rad - metrics.exit_heading_rad)
            )
        )
        can_exit = (
            metrics.turn_angle_deg <= self.exit_angle_deg
            and heading_error_deg <= self.heading_exit_tolerance_deg
        )
        if not can_exit:
            self._exit_candidate_since = None
            return self.mode

        if self._exit_candidate_since is None:
            self._exit_candidate_since = float(now)
        elif float(now) - self._exit_candidate_since >= self.exit_hold_time:
            self.mode = "STRAIGHT"
            self._exit_candidate_since = None
        return self.mode
