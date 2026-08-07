import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from task_orchestrator.motion_mode import IDLE, NAVIGATION, QR_SEARCH, motion_mode_for_state
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
            TaskOrchestrator.NAVIGATING_TO_WORKSHOP: NAVIGATION,
            TaskOrchestrator.SIM_DELIVERY: NAVIGATION,
            TaskOrchestrator.WAITING_SIM: IDLE,
            TaskOrchestrator.COMPLETE: IDLE,
            TaskOrchestrator.ERROR: IDLE,
            TaskOrchestrator.CANCELLED: IDLE,
        }
        self.assertEqual(expected, {state: motion_mode_for_state(state) for state in expected})
        self.assertEqual(IDLE, motion_mode_for_state("UNKNOWN"))
        self.assertEqual(IDLE, motion_mode_for_state(None))


if __name__ == "__main__":
    unittest.main()
