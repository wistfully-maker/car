"""ROS-independent precision parking helpers for the last navigation waypoint.

两段式精准停车：先导航到最终点正后方 ``approach_offset`` 的中间点，再以
低速紧容差爬完最后一小段。纯数学与配置校验，不依赖 ROS。
"""

import math


def yaw_from_quaternion(qz, qw):
    """Return yaw in radians from a planar quaternion (qz, qw)."""
    return 2.0 * math.atan2(qz, qw)


def validate_pose(pose):
    if not isinstance(pose, dict):
        raise ValueError("pose must be an object")
    for field in ("frame_id", "x", "y", "qz", "qw"):
        if field not in pose:
            raise ValueError("pose requires %s" % field)
    if not isinstance(pose["frame_id"], str) or not pose["frame_id"].strip():
        raise ValueError("pose frame_id must be non-blank text")
    for field in ("x", "y", "qz", "qw"):
        value = pose[field]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError("pose %s must be finite" % field)
    norm = math.hypot(pose["qz"], pose["qw"])
    if norm < 1e-9:
        raise ValueError("pose quaternion has zero norm")
    return dict(pose)


def approach_pose(pose, offset):
    """Shift a goal pose BACK along its heading by ``offset`` metres."""
    pose = validate_pose(pose)
    if (
        isinstance(offset, bool)
        or not isinstance(offset, (int, float))
        or not math.isfinite(offset)
        or not 0.0 < offset <= 0.5
    ):
        raise ValueError("approach offset must be in (0, 0.5] metres")
    yaw = yaw_from_quaternion(pose["qz"], pose["qw"])
    result = dict(pose)
    result["x"] = pose["x"] - offset * math.cos(yaw)
    result["y"] = pose["y"] - offset * math.sin(yaw)
    return result


_CREEP_FIELDS = {
    "max_vel_x": 0.5,
    "acc_lim_x": 1.0,
    "max_vel_theta": 1.0,
    "acc_lim_theta": 2.0,
    "xy_goal_tolerance": 0.1,
    "yaw_goal_tolerance": 0.2,
}

_APPROACH_FIELDS = {
    # 中间点段只放宽松朝向容差：中间点不需要精确朝向（最终点才需要），
    # 防止 TEB 在中间点处死磕 yaw 反复超调振荡（实车观测）。
    "yaw_goal_tolerance": 0.8,
}


def approach_parameters(config):
    """Validate the approach (relaxed) TEB subset; return the normalized dict."""
    if not isinstance(config, dict):
        raise ValueError("approach TEB config must be a mapping")
    result = {}
    for field, cap in _APPROACH_FIELDS.items():
        if field not in config:
            raise ValueError("approach TEB config requires %s" % field)
        value = config[field]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.0 < value <= cap
        ):
            raise ValueError(
                "%s must be in (0, %.2f]" % (field, cap)
            )
        result[field] = float(value)
    return result


def creep_parameters(config):
    """Validate the creep TEB parameter subset; return the normalized dict."""
    if not isinstance(config, dict):
        raise ValueError("creep TEB config must be a mapping")
    result = {}
    for field, cap in _CREEP_FIELDS.items():
        if field not in config:
            raise ValueError("creep TEB config requires %s" % field)
        value = config[field]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.0 < value <= cap
        ):
            raise ValueError(
                "%s must be in (0, %.2f]" % (field, cap)
            )
        result[field] = float(value)
    free_goal_vel = config.get("free_goal_vel")
    if free_goal_vel is not None:
        if not isinstance(free_goal_vel, bool):
            raise ValueError("free_goal_vel must be a boolean")
        result["free_goal_vel"] = free_goal_vel
    return result
