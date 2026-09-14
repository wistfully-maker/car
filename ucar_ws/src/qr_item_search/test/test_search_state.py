import unittest

from qr_item_search.search_state import (
    ACTIVE,
    RESTARTABLE,
    InvalidTransition,
    SearchMachine,
)

EXPECTED_ACTIVE = frozenset({
    "INITIAL_SCAN", "TURNING", "SETTLING", "SCANNING",
    "OFFSET_PASS", "WAITING_HTTP",
})

ALL_STATES = frozenset(
    {
        "IDLE",
        "INITIAL_SCAN",
        "TURNING",
        "SETTLING",
        "SCANNING",
        "OFFSET_PASS",
        "WAITING_HTTP",
        "COMPLETE",
        "NOT_FOUND",
        "ERROR",
        "STOPPED",
    }
)


class SearchMachineTest(unittest.TestCase):
    def machine_in(self, state):
        machine = SearchMachine()
        machine.state = state
        return machine

    def test_exposes_immutable_state_sets(self):
        self.assertEqual(EXPECTED_ACTIVE, ACTIVE)
        self.assertEqual(
            frozenset({"IDLE", "COMPLETE", "NOT_FOUND", "ERROR", "STOPPED"}),
            RESTARTABLE,
        )
        self.assertIsInstance(ACTIVE, frozenset)
        self.assertIsInstance(RESTARTABLE, frozenset)
        with self.assertRaises(AttributeError):
            SearchMachine().ACTIVE.add("IDLE")
        with self.assertRaises(AttributeError):
            SearchMachine().ACTIVE = frozenset()

    def test_nominal_station_cycle(self):
        machine = SearchMachine()
        machine.start()
        self.assertEqual("INITIAL_SCAN", machine.state)
        machine.scan_finished()
        self.assertEqual("TURNING", machine.state)
        machine.heading_reached()
        self.assertEqual("SETTLING", machine.state)
        machine.settled()
        self.assertEqual("SCANNING", machine.state)

    def test_all_urls_wait_for_http(self):
        machine = SearchMachine()
        machine.start()
        machine.urls_collected()
        self.assertEqual("WAITING_HTTP", machine.state)

    def test_has_no_continuous_sweep_api(self):
        machine = SearchMachine()
        self.assertFalse(hasattr(machine, "fast_sweep_finished"))
        self.assertFalse(hasattr(machine, "resume_rescan"))
        self.assertFalse(hasattr(machine, "rescan_finished"))
        for old_event in (
            "turn_reached",
            "observation_succeeded",
            "observation_failed",
            "scan_timeout",
            "match_decision",
            "wall_count",
            "wall_index",
        ):
            with self.subTest(old_event=old_event):
                self.assertFalse(hasattr(machine, old_event))
        with self.assertRaises(TypeError):
            SearchMachine(4)

    def test_start_accepts_every_restartable_state(self):
        for source in RESTARTABLE:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                machine.start()
                self.assertEqual("INITIAL_SCAN", machine.state)

    def test_start_rejects_every_active_state(self):
        for source in ACTIVE:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                self.assert_invalid(machine, "start")

    def test_scan_finished_turns_from_initial_or_scanning_only(self):
        legal = {"INITIAL_SCAN", "SCANNING"}
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                if source in legal:
                    machine.scan_finished()
                    self.assertEqual("TURNING", machine.state)
                else:
                    self.assert_invalid(machine, "scan_finished")

    def test_heading_reached_from_turning_only(self):
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                if source == "TURNING":
                    machine.heading_reached()
                    self.assertEqual("SETTLING", machine.state)
                else:
                    self.assert_invalid(machine, "heading_reached")

    def test_settled_from_settling_only(self):
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                if source == "SETTLING":
                    machine.settled()
                    self.assertEqual("SCANNING", machine.state)
                else:
                    self.assert_invalid(machine, "settled")

    def test_urls_collected_waits_for_http_from_scan_states(self):
        legal = {"INITIAL_SCAN", "TURNING", "SETTLING", "SCANNING", "OFFSET_PASS"}
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                if source in legal:
                    machine.urls_collected()
                    self.assertEqual("WAITING_HTTP", machine.state)
                else:
                    self.assert_invalid(machine, "urls_collected")

    def test_pass_finished_enters_offset_pass_once(self):
        machine = self.machine_at_scanning()
        machine.pass_finished()
        self.assertEqual("OFFSET_PASS", machine.state)

    def test_offset_pass_started_resumes_turning(self):
        machine = self.machine_at_scanning()
        machine.pass_finished()
        machine.offset_pass_started()
        self.assertEqual("TURNING", machine.state)

    def test_scan_exhausted_reports_not_found_from_scanning(self):
        machine = self.machine_at_scanning()
        machine.scan_exhausted()
        self.assertEqual("NOT_FOUND", machine.state)

    def test_scan_exhausted_rejected_when_not_in_scanning(self):
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                if source == "SCANNING":
                    continue
                self.assert_invalid(machine, "scan_exhausted")

    def machine_at_scanning(self):
        machine = SearchMachine()
        machine.start()
        machine.scan_finished()
        machine.heading_reached()
        machine.settled()
        return machine

    def test_items_resolved_completes_from_every_active_state_only(self):
        self.assert_active_event("items_resolved", "COMPLETE")

    def test_timeout_reports_not_found_from_every_active_state_only(self):
        self.assert_active_event("timeout", "NOT_FOUND")

    def test_stop_reaches_stopped_from_every_state_including_stopped(self):
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                machine.stop()
                self.assertEqual("STOPPED", machine.state)

    def test_fail_reaches_error_from_every_state(self):
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                machine.fail()
                self.assertEqual("ERROR", machine.state)

    def test_invalid_transition_names_event_and_current_state(self):
        machine = self.machine_in("IDLE")

        with self.assertRaisesRegex(
            InvalidTransition, r"urls_collected.*IDLE|IDLE.*urls_collected"
        ):
            machine.urls_collected()

    def assert_active_event(self, event, destination):
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(event=event, source=source):
                if source in ACTIVE:
                    getattr(machine, event)()
                    self.assertEqual(destination, machine.state)
                else:
                    self.assert_invalid(machine, event)

    def assert_invalid(self, machine, event):
        before = machine.state
        with self.assertRaises(InvalidTransition) as caught:
            getattr(machine, event)()
        self.assertIn(event, str(caught.exception))
        self.assertIn(before, str(caught.exception))
        self.assertEqual(before, machine.state)


if __name__ == "__main__":
    unittest.main()
