import unittest

from qr_item_search.search_state import InvalidTransition, SearchMachine


class SearchMachineTest(unittest.TestCase):
    def test_matching_first_wall_finishes_immediately(self):
        machine = SearchMachine(wall_count=3)

        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.observation_succeeded()
        machine.match_decision(True)

        self.assertEqual("SUCCESS", machine.state)
        self.assertEqual(0, machine.wall_index)

    def test_failed_observation_advances_immediately(self):
        machine = SearchMachine(wall_count=3)
        machine.start()
        machine.turn_reached()
        machine.settled()

        machine.observation_failed()

        self.assertEqual("TURNING", machine.state)
        self.assertEqual(1, machine.wall_index)

    def test_last_wall_timeout_reports_not_found(self):
        machine = SearchMachine(wall_count=1)
        machine.start()
        machine.turn_reached()
        machine.settled()

        machine.scan_timeout()

        self.assertEqual("NOT_FOUND", machine.state)

    def test_last_wall_non_match_reports_not_found(self):
        machine = SearchMachine(wall_count=1)
        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.observation_succeeded()

        machine.match_decision(False)

        self.assertEqual("NOT_FOUND", machine.state)

    def test_stop_returns_to_idle(self):
        machine = SearchMachine(wall_count=3)
        machine.start()

        machine.stop()

        self.assertEqual("IDLE", machine.state)

    def test_rejects_invalid_wall_count(self):
        for wall_count in (0, -1, 1.5, True):
            with self.subTest(wall_count=wall_count):
                with self.assertRaises(ValueError):
                    SearchMachine(wall_count)

    def test_invalid_transition_raises(self):
        machine = SearchMachine(wall_count=3)

        with self.assertRaises(InvalidTransition):
            machine.settled()
        with self.assertRaises(InvalidTransition):
            machine.match_decision("not-a-bool")

        machine.start()
        with self.assertRaises(InvalidTransition):
            machine.start()

    def test_terminal_state_can_start_new_search(self):
        machine = SearchMachine(wall_count=1)
        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.scan_timeout()

        machine.start()

        self.assertEqual("TURNING", machine.state)
        self.assertEqual(0, machine.wall_index)

    def test_match_decision_rejects_non_boolean_without_changing_state(self):
        machine = SearchMachine(wall_count=3)
        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.observation_succeeded()

        for matched in (None, 0, 1, "true", object()):
            with self.subTest(matched=matched):
                with self.assertRaises(TypeError):
                    machine.match_decision(matched)
                self.assertEqual("WAITING_MATCH", machine.state)
                self.assertEqual(0, machine.wall_index)


if __name__ == "__main__":
    unittest.main()
