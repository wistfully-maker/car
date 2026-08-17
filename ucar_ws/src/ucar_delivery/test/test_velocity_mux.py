"""Task 4: ROS-free fail-closed standalone velocity mux.

One mode -> exactly one isolated source. Every mode change returns zero
before the new source is accepted. Stale timestamps, wrong sources,
unknown modes, EMERGENCY_STOP, IDLE, source timeout and shutdown all
fall back to zero.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery.velocity_mux import Command, VelocityMux, ZERO

NAV = Command(linear_x=0.3, angular_z=0.1)
MANUAL = Command(angular_z=0.4)
PARKING = Command(linear_x=0.08, angular_z=-0.2)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ModeSelectionTests(unittest.TestCase):
    def test_navigation_mode_uses_navigation_source(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NAVIGATION", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        self.assertEqual(NAV, mux.output(now=1.1))

    def test_visual_search_mode_uses_manual_source(self):
        mux = VelocityMux(Clock())
        mux.set_mode("VISUAL_SEARCH", now=1.0)
        mux.update("manual", MANUAL, now=1.1)
        self.assertEqual(MANUAL, mux.output(now=1.1))

    def test_parking_mode_uses_parking_source(self):
        mux = VelocityMux(Clock())
        mux.set_mode("PARKING", now=1.0)
        mux.update("parking", PARKING, now=1.1)
        self.assertEqual(PARKING, mux.output(now=1.1))

    def test_idle_mode_returns_zero(self):
        mux = VelocityMux(Clock())
        mux.update("navigation", NAV, now=1.0)
        self.assertEqual(ZERO, mux.output(now=1.1))

    def test_emergency_stop_mode_returns_zero(self):
        mux = VelocityMux(Clock())
        mux.set_mode("PARKING", now=1.0)
        mux.update("parking", PARKING, now=1.1)
        mux.set_mode("EMERGENCY_STOP", now=1.2)
        self.assertEqual(ZERO, mux.output(now=1.2))

    def test_unknown_mode_returns_zero(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NO_SUCH_MODE", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        self.assertEqual(ZERO, mux.output(now=1.1))

    def test_mode_switch_returns_zero_before_new_source(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NAVIGATION", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        self.assertEqual(NAV, mux.output(now=1.1))
        mux.set_mode("PARKING", now=2.0)
        self.assertEqual(ZERO, mux.output(now=2.0))
        mux.update("parking", PARKING, now=2.1)
        self.assertEqual(PARKING, mux.output(now=2.1))


class SourceIsolationTests(unittest.TestCase):
    def test_wrong_source_is_ignored(self):
        mux = VelocityMux(Clock())
        mux.set_mode("PARKING", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        self.assertEqual(ZERO, mux.output(now=1.1))
        mux.update("parking", PARKING, now=1.2)
        self.assertEqual(PARKING, mux.output(now=1.2))

    def test_inactive_source_does_not_clobber_selected_command(self):
        mux = VelocityMux(Clock())
        mux.set_mode("PARKING", now=1.0)
        mux.update("parking", PARKING, now=1.1)
        mux.update("navigation", NAV, now=1.2)
        self.assertEqual(PARKING, mux.output(now=1.2))

    def test_stale_source_update_is_rejected(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NAVIGATION", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        self.assertTrue(
            mux.update("navigation", Command(linear_x=9.9), now=1.05)
            is False
        )
        self.assertEqual(NAV, mux.output(now=1.2))

    def test_unknown_source_is_rejected(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NAVIGATION", now=1.0)
        self.assertFalse(mux.update("qr", NAV, now=1.1))
        self.assertEqual(ZERO, mux.output(now=1.1))


class FailClosedTests(unittest.TestCase):
    def test_source_timeout_returns_zero(self):
        mux = VelocityMux(Clock(), source_timeout=0.3)
        mux.set_mode("NAVIGATION", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        self.assertEqual(NAV, mux.output(now=1.39))
        self.assertEqual(ZERO, mux.output(now=1.41))

    def test_shutdown_returns_zero(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NAVIGATION", now=1.0)
        mux.update("navigation", NAV, now=1.1)
        mux.shutdown(now=1.2)
        self.assertEqual(ZERO, mux.output(now=1.2))

    def test_no_source_yet_returns_zero(self):
        mux = VelocityMux(Clock())
        mux.set_mode("NAVIGATION", now=1.0)
        self.assertEqual(ZERO, mux.output(now=1.1))

    def test_command_is_immutable_tuple(self):
        self.assertEqual(6, len(NAV))
        with self.assertRaises(TypeError):
            NAV[0] = 1.0


if __name__ == "__main__":
    unittest.main()
