"""Fixed-angle observation schedules for stop-and-scan QR search."""

import math


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("%s must be a finite number" % name)
    return float(value)


def build_pass_angles(step_angle_deg, offset_angle_deg=0.0):
    """Return observation angles in [0, 360) ordered counter-clockwise.

    The first pass starts at ``offset_angle_deg`` and visits every
    ``step_angle_deg`` until the circle closes.  ``step_angle_deg`` must
    divide 360 exactly so every pass covers the full circle.
    """
    step = _finite_number(step_angle_deg, "step_angle_deg")
    offset = _finite_number(offset_angle_deg, "offset_angle_deg")
    if step <= 0.0:
        raise ValueError("step_angle_deg must be positive")
    if not 0.0 <= offset < 360.0:
        raise ValueError("offset_angle_deg must be in [0, 360)")
    count = 360.0 / step
    if abs(count - round(count)) > 1e-9:
        raise ValueError("step_angle_deg must divide 360")
    stations = int(round(count))
    return [round((offset + index * step) % 360.0, 9) for index in range(stations)]
