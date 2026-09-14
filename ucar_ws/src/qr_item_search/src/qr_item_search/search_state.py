"""Stop-and-scan QR search state machine (protocol v1 compatible)."""

ACTIVE = frozenset({
    "INITIAL_SCAN", "TURNING", "SETTLING", "SCANNING",
    "OFFSET_PASS", "WAITING_HTTP",
})
RESTARTABLE = frozenset({"IDLE", "COMPLETE", "NOT_FOUND", "ERROR", "STOPPED"})


class InvalidTransition(RuntimeError):
    pass


class SearchMachine:
    __slots__ = ("state",)

    ACTIVE = ACTIVE
    RESTARTABLE = RESTARTABLE

    def __init__(self):
        self.state = "IDLE"

    def start(self):
        self._transition("start", RESTARTABLE, "INITIAL_SCAN")

    def scan_finished(self):
        """Initial station window closed, or a station window closed with a
        next station remaining: begin turning toward the next angle."""
        self._transition(
            "scan_finished",
            frozenset({"INITIAL_SCAN", "SCANNING"}),
            "TURNING",
        )

    def heading_reached(self):
        self._transition(
            "heading_reached",
            frozenset({"TURNING"}),
            "SETTLING",
        )

    def settled(self):
        self._transition("settled", frozenset({"SETTLING"}), "SCANNING")

    def urls_collected(self):
        self._transition(
            "urls_collected",
            frozenset({"INITIAL_SCAN", "TURNING", "SETTLING", "SCANNING", "OFFSET_PASS"}),
            "WAITING_HTTP",
        )

    def pass_finished(self):
        """A pass ended without collecting all URLs; the offset pass follows."""
        self._transition("pass_finished", frozenset({"SCANNING"}), "OFFSET_PASS")

    def offset_pass_started(self):
        """Controller rebuilt the schedule for the offset pass."""
        self._transition("offset_pass_started", frozenset({"OFFSET_PASS"}), "TURNING")

    def scan_exhausted(self):
        """The final pass ended without collecting all URLs."""
        self._transition("scan_exhausted", frozenset({"SCANNING"}), "NOT_FOUND")

    def items_resolved(self):
        self._transition("items_resolved", ACTIVE, "COMPLETE")

    def timeout(self):
        self._transition("timeout", ACTIVE, "NOT_FOUND")

    def stop(self):
        self.state = "STOPPED"

    def fail(self):
        self.state = "ERROR"

    def _transition(self, event, allowed_states, destination):
        if self.state not in allowed_states:
            raise InvalidTransition(
                "{} is invalid while in {}".format(event, self.state)
            )
        self.state = destination
