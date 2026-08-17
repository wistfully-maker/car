"""ROS-free fail-closed standalone velocity mux (Task 4).

Exactly one motion stage owns the final /cmd_vel at any time, selected by
the private /ucar_delivery/motion_mode vocabulary:

    NAVIGATION      -> navigation  (/cmd_vel/delivery_navigation)
    VISUAL_SEARCH   -> manual      (/cmd_vel/delivery_manual)
    PARKING         -> parking     (/cmd_vel/delivery_parking)
    IDLE / EMERGENCY_STOP / unknown -> zero

Every mode change returns zero before the new source is accepted. Any
stale timestamp, wrong source, source timeout, exception, shutdown and
emergency stop falls back to a guaranteed zero command.
"""


class Command(tuple):
    """Immutable six-axis velocity command.

    Order: linear_x, linear_y, linear_z, angular_x, angular_y, angular_z.
    """

    __slots__ = ()

    def __new__(cls, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                angular_x=0.0, angular_y=0.0, angular_z=0.0):
        return tuple.__new__(
            cls,
            (float(linear_x), float(linear_y), float(linear_z),
             float(angular_x), float(angular_y), float(angular_z)),
        )

    @property
    def linear_x(self):
        return self[0]

    @property
    def linear_y(self):
        return self[1]

    @property
    def linear_z(self):
        return self[2]

    @property
    def angular_x(self):
        return self[3]

    @property
    def angular_y(self):
        return self[4]

    @property
    def angular_z(self):
        return self[5]


ZERO = Command()


class VelocityMux:
    NAVIGATION = "NAVIGATION"
    VISUAL_SEARCH = "VISUAL_SEARCH"
    PARKING = "PARKING"
    IDLE = "IDLE"
    EMERGENCY_STOP = "EMERGENCY_STOP"

    SOURCE_BY_MODE = {
        NAVIGATION: "navigation",
        VISUAL_SEARCH: "manual",
        PARKING: "parking",
    }
    VALID_MODES = frozenset(SOURCE_BY_MODE) | frozenset(
        (IDLE, EMERGENCY_STOP)
    )
    SOURCES = frozenset(SOURCE_BY_MODE.values())

    def __init__(self, clock, source_timeout=0.3):
        self._clock = clock
        self._source_timeout = float(source_timeout)
        self._mode = self.IDLE
        self._commands = {}
        self._last_by_source = {}
        self._shutdown = False

    def set_mode(self, mode, now=None):
        """Switch mode; unknown modes fall back to IDLE (zero output)."""
        if mode not in self.VALID_MODES:
            mode = self.IDLE
        self._mode = mode
        # 模式变化立即归零，直到新模式对应的源发布新命令。
        selected = self.SOURCE_BY_MODE.get(mode)
        if selected is not None:
            self._commands.pop(selected, None)

    def update(self, source, command, now=None):
        """Accept a command from an isolated source.

        Rejects unknown sources and timestamps older than the last
        accepted one for that source.
        """
        if source not in self.SOURCES:
            return False
        now = self._clock() if now is None else now
        previous = self._last_by_source.get(source)
        if previous is not None and now < previous:
            return False
        self._last_by_source[source] = now
        self._commands[source] = (command, now)
        return True

    def output(self, now=None):
        """The command for the current mode, or zero when anything is off."""
        now = self._clock() if now is None else now
        if self._shutdown:
            return ZERO
        expected = self.SOURCE_BY_MODE.get(self._mode)
        if expected is None:
            return ZERO
        selected = self._commands.get(expected)
        if selected is None:
            return ZERO
        command, updated_at = selected
        if now - updated_at > self._source_timeout:
            return ZERO
        return command

    def shutdown(self, now=None):
        self._shutdown = True
        self._commands.clear()
