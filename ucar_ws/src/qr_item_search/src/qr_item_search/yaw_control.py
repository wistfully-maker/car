import math


def normalize_angle(angle):
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_command(current, target, kp, max_speed, tolerance):
    if not all(
        math.isfinite(value)
        for value in (current, target, kp, max_speed, tolerance)
    ):
        raise ValueError("control parameters must be finite")
    if kp <= 0:
        raise ValueError("kp must be positive")
    if max_speed <= 0:
        raise ValueError("max_speed must be positive")
    if tolerance < 0:
        raise ValueError("tolerance must be non-negative")

    error = normalize_angle(target - current)
    if abs(error) <= tolerance:
        return 0.0, True

    speed = kp * error
    speed = max(-max_speed, min(max_speed, speed))
    return speed, False
