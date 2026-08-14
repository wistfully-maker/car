import math


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("%s must be a finite number" % name)
    return float(value)


def normalize_angle(angle):
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_command(
    current,
    target,
    kp,
    max_speed,
    tolerance,
    min_speed=0.0,
):
    if not all(
        math.isfinite(value)
        for value in (
            current,
            target,
            kp,
            max_speed,
            tolerance,
            min_speed,
        )
    ):
        raise ValueError("control parameters must be finite")
    if kp <= 0:
        raise ValueError("kp must be positive")
    if max_speed <= 0:
        raise ValueError("max_speed must be positive")
    if tolerance < 0:
        raise ValueError("tolerance must be non-negative")
    if min_speed < 0 or min_speed > max_speed:
        raise ValueError("min_speed must be between zero and max_speed")

    error = normalize_angle(target - current)
    if abs(error) <= tolerance:
        return 0.0, True

    speed = kp * error
    speed = max(-max_speed, min(max_speed, speed))
    if abs(speed) < min_speed:
        speed = math.copysign(min_speed, error)
    return speed, False


def directed_angular_command(error, speed, tolerance, min_speed=0.0):
    """Return a bounded velocity that follows an already-directed error."""
    error = _finite_number(error, "error")
    speed = _finite_number(speed, "speed")
    tolerance = _finite_number(tolerance, "tolerance")
    min_speed = _finite_number(min_speed, "min_speed")
    if speed <= 0:
        raise ValueError("speed must be positive")
    if tolerance < 0:
        raise ValueError("tolerance must be non-negative")
    if min_speed < 0 or min_speed > speed:
        raise ValueError("min_speed must be between zero and speed")
    if abs(error) <= tolerance:
        return 0.0, True
    magnitude = max(min_speed, min(speed, abs(error)))
    return math.copysign(magnitude, error), False


def staged_angular_command(error, cruise_speed, approach_speed,
                           approach_zone_rad, tolerance_rad):
    """Return (speed, reached) for fixed-angle station approach.

    Within ``tolerance_rad`` the command is zero and the station is reached;
    inside ``approach_zone_rad`` the command is ``approach_speed`` in the
    direction of the error (including a low-speed reverse when the target
    was overshot); otherwise ``cruise_speed`` is used.
    """
    error = _finite_number(error, "error")
    cruise_speed = _finite_number(cruise_speed, "cruise_speed")
    approach_speed = _finite_number(approach_speed, "approach_speed")
    approach_zone_rad = _finite_number(approach_zone_rad, "approach_zone_rad")
    tolerance_rad = _finite_number(tolerance_rad, "tolerance_rad")
    if cruise_speed <= 0 or approach_speed <= 0:
        raise ValueError("speeds must be positive")
    if tolerance_rad < 0 or approach_zone_rad < 0:
        raise ValueError("zones must be non-negative")
    if approach_zone_rad < tolerance_rad:
        raise ValueError("approach_zone_rad must not be smaller than tolerance_rad")
    if abs(error) <= tolerance_rad:
        return 0.0, True
    if abs(error) <= approach_zone_rad:
        magnitude = approach_speed
    else:
        magnitude = cruise_speed
    return math.copysign(magnitude, error), False
