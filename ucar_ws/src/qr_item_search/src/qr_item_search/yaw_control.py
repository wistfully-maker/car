import math


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
