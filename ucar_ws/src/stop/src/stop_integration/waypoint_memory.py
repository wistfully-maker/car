"""ROS-independent workshop observations and Phase 2 waypoint routing."""


def _validate_waypoint_index(waypoint_index, waypoint_count, field):
    if (
        isinstance(waypoint_index, bool)
        or not isinstance(waypoint_index, int)
        or waypoint_index < 0
        or waypoint_index >= waypoint_count
    ):
        raise ValueError("%s must be a valid waypoint index" % field)
    return waypoint_index


class WaypointMemory(object):
    """Remember canonical workshop labels seen at each scan waypoint."""

    def __init__(self, waypoint_count):
        if (
            isinstance(waypoint_count, bool)
            or not isinstance(waypoint_count, int)
            or waypoint_count < 1
        ):
            raise ValueError("waypoint_count must be a positive integer")
        self.waypoint_count = waypoint_count
        self.reset()

    def reset(self):
        self._workshop_candidates = {}
        self._scanned_waypoints = set()

    @property
    def scanned_waypoints(self):
        return set(self._scanned_waypoints)

    def record(self, workshop, waypoint_index):
        waypoint_index = _validate_waypoint_index(
            waypoint_index, self.waypoint_count, "waypoint_index"
        )
        if not isinstance(workshop, str) or not workshop.strip():
            raise ValueError("workshop must be non-empty text")
        workshop = workshop.strip()
        self._workshop_candidates.setdefault(workshop, set()).add(waypoint_index)

    def mark_scanned(self, waypoint_index):
        waypoint_index = _validate_waypoint_index(
            waypoint_index, self.waypoint_count, "waypoint_index"
        )
        self._scanned_waypoints.add(waypoint_index)

    def candidates(self, workshop):
        return set(self._workshop_candidates.get(workshop, set()))

    def unique_waypoint(self, workshop):
        candidates = self.candidates(workshop)
        if len(candidates) != 1:
            return None
        return next(iter(candidates))


def build_phase2_route(physical_waypoint, preferred_waypoint, waypoint_count):
    """Return Phase 2 scan order, excluding the physical parking waypoint."""
    if (
        isinstance(waypoint_count, bool)
        or not isinstance(waypoint_count, int)
        or waypoint_count < 2
    ):
        raise ValueError("waypoint_count must be an integer of at least two")
    physical_waypoint = _validate_waypoint_index(
        physical_waypoint, waypoint_count, "physical_waypoint"
    )
    if preferred_waypoint is not None:
        preferred_waypoint = _validate_waypoint_index(
            preferred_waypoint, waypoint_count, "preferred_waypoint"
        )

    fallback = [
        (physical_waypoint + offset) % waypoint_count
        for offset in range(1, waypoint_count)
    ]
    if preferred_waypoint is None or preferred_waypoint == physical_waypoint:
        return fallback
    return [preferred_waypoint] + [
        waypoint for waypoint in fallback if waypoint != preferred_waypoint
    ]
