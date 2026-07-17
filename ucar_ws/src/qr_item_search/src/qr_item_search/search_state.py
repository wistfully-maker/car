class InvalidTransition(RuntimeError):
    pass


class SearchMachine:
    _TERMINAL_STATES = {"SUCCESS", "NOT_FOUND", "ERROR"}

    def __init__(self, wall_count):
        if type(wall_count) is not int or wall_count <= 0:
            raise ValueError("wall_count must be a positive integer")
        self.wall_count = wall_count
        self.wall_index = 0
        self.state = "IDLE"

    def start(self):
        if self.state != "IDLE" and self.state not in self._TERMINAL_STATES:
            self._invalid("start")
        self.wall_index = 0
        self.state = "TURNING"

    def turn_reached(self):
        self._require("TURNING", "turn_reached")
        self.state = "SETTLING"

    def settled(self):
        self._require("SETTLING", "settled")
        self.state = "SCANNING"

    def observation_succeeded(self):
        self._require("SCANNING", "observation_succeeded")
        self.state = "WAITING_MATCH"

    def observation_failed(self):
        self._require("SCANNING", "observation_failed")
        self._advance()

    def scan_timeout(self):
        self._require("SCANNING", "scan_timeout")
        self._advance()

    def match_decision(self, matched):
        self._require("WAITING_MATCH", "match_decision")
        if type(matched) is not bool:
            raise TypeError("matched must be a bool")
        if matched:
            self.state = "SUCCESS"
        else:
            self._advance()

    def stop(self):
        self.state = "IDLE"

    def fail(self):
        self.state = "ERROR"

    def _advance(self):
        if self.wall_index + 1 >= self.wall_count:
            self.state = "NOT_FOUND"
        else:
            self.wall_index += 1
            self.state = "TURNING"

    def _require(self, expected, event):
        if self.state != expected:
            self._invalid(event)

    def _invalid(self, event):
        raise InvalidTransition(
            "{} is invalid while in {}".format(event, self.state)
        )
