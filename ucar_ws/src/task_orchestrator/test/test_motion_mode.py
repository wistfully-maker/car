import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from task_orchestrator.motion_mode import (
    IDLE,
    LINE_FOLLOW,
    NAVIGATION,
    QR_SEARCH,
    STOP_NAVIGATION,
    motion_mode_for_state,
)
from task_orchestrator.orchestrator import TaskOrchestrator


class MotionModeTests(unittest.TestCase):
    def test_maps_every_declared_state_and_unknown_safely(self):
        expected = {
            TaskOrchestrator.IDLE: IDLE,
            TaskOrchestrator.CHECKING_DEPENDENCIES: IDLE,
            TaskOrchestrator.NAVIGATING_TO_PICKUP: NAVIGATION,
            TaskOrchestrator.WAITING_QR: QR_SEARCH,
            TaskOrchestrator.WAITING_LLM: IDLE,
            TaskOrchestrator.WAITING_SPEECH: IDLE,
            TaskOrchestrator.NAVIGATING_TO_WORKSHOP: STOP_NAVIGATION,
            TaskOrchestrator.NAVIGATING_TO_SIM_WORKSHOP: STOP_NAVIGATION,
            TaskOrchestrator.NAVIGATING_LINE_START: STOP_NAVIGATION,
            TaskOrchestrator.WAITING_LINE_DIRECTION: IDLE,
            TaskOrchestrator.LINE_FOLLOWING: LINE_FOLLOW,
            TaskOrchestrator.WAITING_FINAL_SPEECH: IDLE,
            TaskOrchestrator.COMPLETE: IDLE,
            TaskOrchestrator.ERROR: IDLE,
            TaskOrchestrator.CANCELLED: IDLE,
        }
        self.assertEqual(expected, {state: motion_mode_for_state(state) for state in expected})
        self.assertEqual(IDLE, motion_mode_for_state("UNKNOWN"))
        self.assertEqual(IDLE, motion_mode_for_state(None))

    def test_stop_navigation_is_a_declared_mode(self):
        self.assertEqual("STOP_NAVIGATION", STOP_NAVIGATION)
        self.assertIn(STOP_NAVIGATION, (
            IDLE, NAVIGATION, QR_SEARCH, STOP_NAVIGATION, LINE_FOLLOW,
        ))

    def test_phase3_states_map_to_expected_modes(self):
        self.assertEqual(STOP_NAVIGATION, motion_mode_for_state("NAVIGATING_LINE_START"))
        self.assertEqual(IDLE, motion_mode_for_state("WAITING_LINE_DIRECTION"))
        self.assertEqual(LINE_FOLLOW, motion_mode_for_state("LINE_FOLLOWING"))
        self.assertEqual(IDLE, motion_mode_for_state("WAITING_FINAL_SPEECH"))

    def test_line_follow_is_a_declared_mode(self):
        self.assertEqual("LINE_FOLLOW", LINE_FOLLOW)
        self.assertIn(LINE_FOLLOW, (
            IDLE, NAVIGATION, QR_SEARCH, STOP_NAVIGATION, LINE_FOLLOW,
        ))


if __name__ == "__main__":
    unittest.main()
