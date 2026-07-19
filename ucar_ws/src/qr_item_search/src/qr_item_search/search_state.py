ACTIVE = frozenset({"FAST_SWEEP", "WAITING_HTTP", "TARGETED_RESCAN"})
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
        self._transition("start", RESTARTABLE, "FAST_SWEEP")

    def urls_collected(self):
        self._transition(
            "urls_collected",
            frozenset({"FAST_SWEEP", "TARGETED_RESCAN"}),
            "WAITING_HTTP",
        )

    def fast_sweep_finished(self):
        self._transition(
            "fast_sweep_finished",
            frozenset({"FAST_SWEEP"}),
            "TARGETED_RESCAN",
        )

    def resume_rescan(self):
        self._transition(
            "resume_rescan",
            frozenset({"WAITING_HTTP"}),
            "TARGETED_RESCAN",
        )

    def items_resolved(self):
        self._transition("items_resolved", ACTIVE, "COMPLETE")

    def rescan_finished(self):
        self._transition(
            "rescan_finished",
            frozenset({"TARGETED_RESCAN"}),
            "NOT_FOUND",
        )

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
