"""Pure omni TEB parameter handling for the fast-nav injection applier.

本模块不导入 rospy/dynamic_reconfigure，也不依赖任何 ROS 消息类型；
只负责校验 omni 参数集并返回 TEB dynamic_reconfigure 可用的参数名。

注意：``holonomic_robot`` 不在注入集里。车端部署的 TEB 只在节点启动时
从 rosparam 读取该参数，dynamic_reconfigure 不声明它（update 会被拒绝）。
fast-nav 段的近似全向行为由 ``weight_kinematics_nh=1.0``（软非完整约束压到
可忽略）加横移速度上限实现；stop 栈在自己的 YAML 里以启动参数方式启用
真正的 holonomic 模式。
"""

import math

_REQUIRED_FIELDS = (
    "max_vel_x",
    "max_vel_x_backwards",
    "max_vel_y",
    "max_vel_theta",
    "acc_lim_x",
    "acc_lim_y",
    "acc_lim_theta",
    "weight_kinematics_nh",
    "weight_kinematics_forward_drive",
    "weight_optimaltime",
    "min_obstacle_dist",
    "inflation_dist",
    "weight_obstacle",
    "costmap_obstacles_behind_robot_dist",
)

_HARD_CAPS = {
    "max_vel_x": 1.0,
    "max_vel_x_backwards": 0.5,
    "max_vel_y": 0.6,
    "max_vel_theta": 1.5,
    "acc_lim_x": 1.0,
    "acc_lim_y": 0.8,
    "acc_lim_theta": 2.0,
    "min_obstacle_dist": 0.5,
    "inflation_dist": 1.0,
    "weight_obstacle": 500.0,
    "costmap_obstacles_behind_robot_dist": 5.0,
}


def _positive_finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(float(value)) and float(value) > 0
    except (TypeError, ValueError):
        return False


def validate_omni_teb(config):
    """Validate a raw omni TEB mapping; return the ordered subset of fields.

    任何缺字段、非正数、非有限值或超硬上限都抛 ValueError，保证不会把无效值
    注入 live move_base。注入集以外的键（含启动专用参数 holonomic_robot）
    被静默忽略，不参与运行时更新。
    """
    if not isinstance(config, dict):
        raise ValueError("omni TEB config must be a mapping")
    for field in _REQUIRED_FIELDS:
        if field not in config:
            raise ValueError("missing omni TEB field: %s" % field)
        value = config[field]
        if not _positive_finite(value):
            raise ValueError("%s must be a positive finite number" % field)
        value = float(value)
        if field in _HARD_CAPS and value > _HARD_CAPS[field]:
            raise ValueError(
                "%s=%.3f exceeds hard cap %.3f"
                % (field, value, _HARD_CAPS[field])
            )
    return {field: config[field] for field in _REQUIRED_FIELDS}


def teb_parameters(config):
    """Return the exact parameter set for TEB dynamic_reconfigure."""
    return validate_omni_teb(config)


_CORNER_FIELDS = (
    "max_vel_x",
    "max_vel_x_backwards",
    "max_vel_y",
    "acc_lim_y",
    "max_vel_theta",
    "acc_lim_theta",
)


def validate_corner_set(config):
    """Validate the CORNER speed subset; same positivity and hard caps."""
    if not isinstance(config, dict):
        raise ValueError("corner TEB config must be a mapping")
    result = {}
    for field in _CORNER_FIELDS:
        if field not in config:
            raise ValueError("missing corner TEB field: %s" % field)
        value = config[field]
        if not _positive_finite(value):
            raise ValueError("%s must be a positive finite number" % field)
        value = float(value)
        if field in _HARD_CAPS and value > _HARD_CAPS[field]:
            raise ValueError(
                "%s=%.3f exceeds hard cap %.3f"
                % (field, value, _HARD_CAPS[field])
            )
        result[field] = value
    return result


def ensure_corner_is_slower(straight, corner):
    """CORNER 速度必须严格不高于 STRAIGHT，防止弯道反向加速。"""
    straight = validate_omni_teb(straight)
    corner = validate_corner_set(corner)
    for field in ("max_vel_x", "max_vel_x_backwards", "max_vel_theta"):
        if corner[field] > straight[field]:
            raise ValueError(
                "corner %s=%.3f must not exceed straight %.3f"
                % (field, corner[field], straight[field])
            )
    for field in ("max_vel_y", "acc_lim_y", "acc_lim_theta"):
        if corner[field] > straight[field]:
            raise ValueError(
                "corner %s=%.3f must not exceed straight %.3f"
                % (field, corner[field], straight[field])
            )
    return corner
