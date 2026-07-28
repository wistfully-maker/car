"""Stable piecewise-linear corner extraction for global navigation paths."""

from dataclasses import dataclass
import math

from .corner_geometry import normalize_angle, resample_polyline


@dataclass(frozen=True)
class LineFit:
    heading: float
    length: float
    max_residual: float
    point_count: int


@dataclass(frozen=True)
class PlannedCorner:
    path_distance: float
    point: tuple
    entry_heading: float
    exit_heading: float
    turn_angle: float
    entry_fit: LineFit
    exit_fit: LineFit
    confident: bool


class CornerQueue:
    """Corner progress that survives global-path revisions."""

    def __init__(self, match_distance=0.25, match_heading=math.radians(20.0)):
        self.match_distance = float(match_distance)
        self.match_heading = float(match_heading)
        self._completed = []
        self._pending = []
        self._index = 0

    def reset(self):
        self._completed = []
        self._pending = []
        self._index = 0

    def _matches_completed(self, candidate):
        return any(
            math.hypot(
                candidate.point[0] - completed.point[0],
                candidate.point[1] - completed.point[1],
            )
            <= self.match_distance
            and abs(
                normalize_angle(
                    candidate.exit_heading - completed.exit_heading
                )
            )
            <= self.match_heading
            for completed in self._completed
        )

    def replace(self, corners):
        self._pending = [
            corner
            for corner in corners
            if not self._matches_completed(corner)
        ]
        self._index = 0

    def current(self):
        if self._index >= len(self._pending):
            return None
        return self._pending[self._index]

    def current_identity(self):
        corner = self.current()
        if corner is None:
            return None
        return (
            round(corner.point[0] / 0.05),
            round(corner.point[1] / 0.05),
            round(corner.exit_heading / math.radians(10.0)),
        )

    def complete_current(self):
        corner = self.current()
        if corner is not None:
            self._completed.append(corner)
            self._index += 1

    def skip_current(self):
        if self.current() is not None:
            self._index += 1

    def remaining_count(self):
        return max(0, len(self._pending) - self._index)


def _point_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.hypot(point[0] - projection[0], point[1] - projection[1])


def _rdp_indices(points, tolerance):
    if len(points) <= 2:
        return list(range(len(points)))

    kept = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        best_index = None
        best_distance = tolerance
        for index in range(start + 1, end):
            distance = _point_segment_distance(
                points[index], points[start], points[end]
            )
            if distance > best_distance:
                best_index = index
                best_distance = distance
        if best_index is not None:
            kept.add(best_index)
            stack.append((start, best_index))
            stack.append((best_index, end))
    return sorted(kept)


def _fit_line(points):
    start = points[0]
    end = points[-1]
    length = sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    xx = sum((point[0] - center_x) ** 2 for point in points)
    yy = sum((point[1] - center_y) ** 2 for point in points)
    xy = sum(
        (point[0] - center_x) * (point[1] - center_y) for point in points
    )
    heading = 0.5 * math.atan2(2.0 * xy, xx - yy)
    forward = math.atan2(end[1] - start[1], end[0] - start[0])
    if abs(normalize_angle(heading - forward)) > math.pi / 2.0:
        heading = normalize_angle(heading + math.pi)
    normal_x = -math.sin(heading)
    normal_y = math.cos(heading)
    residual = max(
        abs(
            (point[0] - center_x) * normal_x
            + (point[1] - center_y) * normal_y
        )
        for point in points
    )
    return LineFit(heading, length, residual, len(points))


def _cumulative_distances(points):
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(
            distances[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    return distances


def extract_corner_plan(
    points,
    simplify_tolerance,
    min_corner_angle,
    min_segment_length,
    max_fit_residual,
    same_turn_merge_distance,
    resample_spacing=0.05,
):
    clean = []
    for point in points:
        point = (float(point[0]), float(point[1]))
        if not clean or math.hypot(
            point[0] - clean[-1][0], point[1] - clean[-1][1]
        ) > 1.0e-9:
            clean.append(point)
    if len(clean) < 3:
        return []
    clean = resample_polyline(clean, resample_spacing)

    cumulative = _cumulative_distances(clean)
    simplified = _rdp_indices(clean, simplify_tolerance)
    candidates = []
    for position in range(1, len(simplified) - 1):
        previous_index = simplified[position - 1]
        corner_index = simplified[position]
        next_index = simplified[position + 1]
        entry_fit = _fit_line(clean[previous_index : corner_index + 1])
        exit_fit = _fit_line(clean[corner_index : next_index + 1])
        turn_angle = normalize_angle(
            exit_fit.heading - entry_fit.heading
        )
        if abs(turn_angle) < min_corner_angle:
            continue
        confident = (
            entry_fit.length >= min_segment_length
            and exit_fit.length >= min_segment_length
            and entry_fit.max_residual <= max_fit_residual
            and exit_fit.max_residual <= max_fit_residual
        )
        candidates.append(
            PlannedCorner(
                path_distance=cumulative[corner_index],
                point=clean[corner_index],
                entry_heading=entry_fit.heading,
                exit_heading=exit_fit.heading,
                turn_angle=turn_angle,
                entry_fit=entry_fit,
                exit_fit=exit_fit,
                confident=confident,
            )
        )

    merged = []
    for candidate in candidates:
        if (
            merged
            and candidate.turn_angle * merged[-1].turn_angle > 0.0
            and candidate.path_distance - merged[-1].path_distance
            < same_turn_merge_distance
        ):
            if abs(candidate.turn_angle) > abs(merged[-1].turn_angle):
                merged[-1] = candidate
        else:
            merged.append(candidate)
    return merged
