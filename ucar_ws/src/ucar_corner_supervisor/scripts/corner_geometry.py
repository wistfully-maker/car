#!/usr/bin/env python3

"""Pure geometry and control primitives used by the ROS supervisor node."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CornerObservation:
    distance: float
    turn_angle: float
    exit_heading: float
    point: tuple


def normalize_angle(angle):
    """Return an angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _deduplicate(points, epsilon=1.0e-9):
    result = []
    for point in points:
        current = (float(point[0]), float(point[1]))
        if not result or math.hypot(
            current[0] - result[-1][0], current[1] - result[-1][1]
        ) > epsilon:
            result.append(current)
    return result


def resample_polyline(points, spacing):
    """Resample a polyline without allowing steps larger than ``spacing``."""
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    source = _deduplicate(points)
    if len(source) < 2:
        return source

    sampled = [source[0]]
    remaining = spacing
    start = source[0]
    for end in source[1:]:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        while segment_length + 1.0e-12 >= remaining:
            ratio = remaining / segment_length
            start = (start[0] + ratio * dx, start[1] + ratio * dy)
            sampled.append(start)
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            segment_length = math.hypot(dx, dy)
            remaining = spacing
        remaining -= segment_length
        start = end

    if math.hypot(
        source[-1][0] - sampled[-1][0], source[-1][1] - sampled[-1][1]
    ) > 1.0e-9:
        sampled.append(source[-1])
    return sampled


def find_first_corner(
    points,
    search_distance,
    min_angle,
    spacing=0.05,
    direction_window=0.20,
):
    """Find the first significant direction change along a local path slice."""
    sampled = resample_polyline(points, spacing)
    window_steps = max(1, int(round(direction_window / spacing)))
    if len(sampled) < 2 * window_steps + 1:
        return None

    cumulative = [0.0]
    for first, second in zip(sampled, sampled[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )

    candidates = []
    for index in range(window_steps, len(sampled) - window_steps):
        if cumulative[index] > search_distance:
            break
        before = sampled[index - window_steps]
        apex = sampled[index]
        after = sampled[index + window_steps]
        entry_heading = math.atan2(apex[1] - before[1], apex[0] - before[0])
        exit_heading = math.atan2(after[1] - apex[1], after[0] - apex[0])
        turn_angle = normalize_angle(exit_heading - entry_heading)
        if abs(turn_angle) >= min_angle:
            candidates.append((index, turn_angle, exit_heading))

    if not candidates:
        return None

    first_cluster = [candidates[0]]
    for candidate in candidates[1:]:
        if candidate[0] > first_cluster[-1][0] + 1:
            break
        first_cluster.append(candidate)
    index, turn_angle, exit_heading = max(
        first_cluster, key=lambda candidate: abs(candidate[1])
    )
    return CornerObservation(
        distance=cumulative[index],
        turn_angle=turn_angle,
        exit_heading=exit_heading,
        point=sampled[index],
    )
