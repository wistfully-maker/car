import os
import sys
import tempfile
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from line_follow_integration.runtime import ImageHealthGate, LineFollowSession


class FakeClock:
    def __init__(self, value):
        self.value = float(value)

    def __call__(self):
        return self.value


def write_fresh(path, text, timestamp):
    path.write_text(text, encoding="utf-8")
    os.utime(path, (timestamp, timestamp))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_direction_session_waits_for_red_then_locks_left(self):
        clock = FakeClock(100.0)
        result = self.root / "yolo.txt"
        session = LineFollowSession(str(result), str(self.root / "done"), clock, 30.0, 120.0)
        session.start("task-1", "line-1")
        write_fresh(result, "red_light", 101.0)
        self.assertEqual(("waiting_signal", None), session.poll_direction())
        write_fresh(result, "left_turn", 102.0)
        self.assertEqual(("direction_selected", "left_turn"), session.poll_direction())
        self.assertEqual(("direction_selected", "left_turn"), session.poll_direction())

    def test_direction_timeout_defaults_to_straight(self):
        clock = FakeClock(10.0)
        session = LineFollowSession(str(self.root / "yolo"), str(self.root / "done"), clock, 30.0, 120.0)
        session.start("task-1", "line-1")
        clock.value = 40.0
        self.assertEqual(("direction_selected", "straight"), session.poll_direction())

    def test_stale_stop_file_never_succeeds(self):
        done = self.root / "done"
        write_fresh(done, "done", 9.0)
        clock = FakeClock(10.0)
        session = LineFollowSession(str(self.root / "yolo"), str(done), clock, 30.0, 120.0)
        session.start("task-1", "line-1")
        session.lock_direction("straight")
        self.assertEqual(("following", None), session.poll_follow(child_running=True))

    def test_image_gate_stops_then_recovers_before_grace(self):
        clock = FakeClock(10.0)
        gate = ImageHealthGate(clock, max_age=0.5, recovery_grace=3.0)
        gate.observe_frame(10.0)
        self.assertTrue(gate.allows_motion())
        clock.value = 10.6
        self.assertEqual("stop", gate.poll())
        self.assertFalse(gate.allows_motion())
        clock.value = 12.0
        gate.observe_frame(12.0)
        self.assertEqual("healthy", gate.poll())
        self.assertTrue(gate.allows_motion())

    def test_image_gate_fails_after_recovery_grace(self):
        clock = FakeClock(20.0)
        gate = ImageHealthGate(clock, max_age=0.5, recovery_grace=3.0)
        gate.observe_frame(20.0)
        clock.value = 20.6
        self.assertEqual("stop", gate.poll())
        clock.value = 23.6
        self.assertEqual("failure", gate.poll())


if __name__ == "__main__":
    unittest.main()
