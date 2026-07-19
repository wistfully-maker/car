import unittest

from qr_item_search.search_state import (
    ACTIVE,
    RESTARTABLE,
    InvalidTransition,
    SearchMachine,
)


ALL_STATES = frozenset(
    {
        "IDLE",
        "FAST_SWEEP",
        "WAITING_HTTP",
        "TARGETED_RESCAN",
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
        self.assertEqual(
            frozenset({"FAST_SWEEP", "WAITING_HTTP", "TARGETED_RESCAN"}),
            ACTIVE,
        )
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

    def test_has_no_fixed_wall_api(self):
        machine = SearchMachine()

        self.assertFalse(hasattr(machine, "wall_count"))
        self.assertFalse(hasattr(machine, "wall_index"))
        for old_event in (
            "turn_reached",
            "settled",
            "observation_succeeded",
            "observation_failed",
            "scan_timeout",
            "match_decision",
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
                self.assertEqual("FAST_SWEEP", machine.state)

    def test_start_rejects_every_active_state(self):
        for source in ACTIVE:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                self.assert_invalid(machine, "start")

    def test_single_source_events_accept_only_their_legal_sources(self):
        cases = (
            ("fast_sweep_finished", {"FAST_SWEEP"}, "TARGETED_RESCAN"),
            ("resume_rescan", {"WAITING_HTTP"}, "TARGETED_RESCAN"),
            ("rescan_finished", {"TARGETED_RESCAN"}, "NOT_FOUND"),
        )
        for event, sources, destination in cases:
            for source in ALL_STATES:
                machine = self.machine_in(source)
                with self.subTest(event=event, source=source):
                    if source in sources:
                        getattr(machine, event)()
                        self.assertEqual(destination, machine.state)
                    else:
                        self.assert_invalid(machine, event)

    def test_urls_collected_accepts_both_sweep_states_only(self):
        legal = {"FAST_SWEEP", "TARGETED_RESCAN"}
        for source in ALL_STATES:
            machine = self.machine_in(source)
            with self.subTest(source=source):
                if source in legal:
                    machine.urls_collected()
                    self.assertEqual("WAITING_HTTP", machine.state)
                else:
                    self.assert_invalid(machine, "urls_collected")

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
