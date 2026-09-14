"""ROS-independent system readiness evaluation."""


EXPECTED_GLOBAL_PLANNER = "global_planner/GlobalPlanner"
EXPECTED_LOCAL_PLANNER = "teb_local_planner/TebLocalPlannerROS"


class ReadinessSnapshot:
    def __init__(
        self,
        now,
        scan_stamp=None,
        odom_stamp=None,
        map_received=False,
        tf_map_odom=False,
        tf_odom_base=False,
        tf_base_laser=False,
        move_base_available=False,
        lidar_loc_live=False,
        amcl_live=False,
        global_planner=None,
        local_planner=None,
    ):
        self.now = now
        self.scan_stamp = scan_stamp
        self.odom_stamp = odom_stamp
        self.map_received = map_received
        self.tf_map_odom = tf_map_odom
        self.tf_odom_base = tf_odom_base
        self.tf_base_laser = tf_base_laser
        self.move_base_available = move_base_available
        self.lidar_loc_live = lidar_loc_live
        self.amcl_live = amcl_live
        self.global_planner = global_planner
        self.local_planner = local_planner


def _freshness_missing(name, stamp, now, max_age):
    if stamp is None:
        return "%s not received" % name
    age = now - stamp
    if age < 0:
        return "%s timestamp is in the future" % name
    if age > max_age:
        return "%s stale (age %.3fs)" % (name, age)
    return None


def missing_requirements(snapshot, max_age):
    """Return every missing requirement in deterministic diagnostic order."""
    missing = []
    for name, stamp in (
        ("scan", snapshot.scan_stamp),
        ("odom", snapshot.odom_stamp),
    ):
        reason = _freshness_missing(name, stamp, snapshot.now, max_age)
        if reason:
            missing.append(reason)
    if not snapshot.map_received:
        missing.append("map not received")
    for available, description in (
        (snapshot.tf_map_odom, "TF map->odom unavailable"),
        (snapshot.tf_odom_base, "TF odom->base_link unavailable"),
        (snapshot.tf_base_laser, "TF base_link->laser_frame unavailable"),
        (snapshot.move_base_available, "move_base action unavailable"),
        (snapshot.lidar_loc_live, "/lidar_loc is not live"),
    ):
        if not available:
            missing.append(description)
    if snapshot.amcl_live:
        missing.append("/amcl conflicts with /lidar_loc")
    if snapshot.global_planner != EXPECTED_GLOBAL_PLANNER:
        missing.append(
            "global planner mismatch: expected %s, got %s"
            % (EXPECTED_GLOBAL_PLANNER, snapshot.global_planner or "<unset>")
        )
    if snapshot.local_planner != EXPECTED_LOCAL_PLANNER:
        missing.append(
            "local planner mismatch: expected %s, got %s"
            % (EXPECTED_LOCAL_PLANNER, snapshot.local_planner or "<unset>")
        )
    return missing
