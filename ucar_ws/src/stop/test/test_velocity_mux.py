import math
import sys
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from stop_integration.velocity_mux import VelocityMux, TwistValue, VectorValue


def twist(x=0.0, z=0.0):
    return TwistValue(linear=VectorValue(x=x), angular=VectorValue(z=z))


class VelocityMuxModeTests(unittest.TestCase):
    def test_starts_idle_and_ignores_every_source(self):
        mux = VelocityMux()
        self.assertEqual("IDLE", mux.mode)
        self.assertIsNone(mux.accept("navigation", twist(0.5), 1.0))
        self.assertIsNone(mux.accept("manual", twist(0.3), 1.0))

    def test_mode_change_publishes_zero_before_new_source(self):
        mux = VelocityMux()
        first = mux.set_mode("NAVIGATION", 0.0)
        self.assertIsNotNone(first)
        self.assertEqual(0.0, first.linear.x)
        forwarded = mux.accept("navigation", twist(0.5), 1.0)
        self.assertEqual(0.5, forwarded.linear.x)
        second = mux.set_mode("MANUAL", 2.0)
        self.assertIsNotNone(second)
        self.assertEqual(0.0, second.linear.x)
        self.assertEqual(0.3, mux.accept("manual", twist(0.3), 3.0).linear.x)

    def test_same_mode_returns_none(self):
        mux = VelocityMux()
        mux.set_mode("NAVIGATION", 0.0)
        self.assertIsNone(mux.set_mode("NAVIGATION", 1.0))

    def test_wrong_source_is_ignored(self):
        mux = VelocityMux()
        mux.set_mode("NAVIGATION", 0.0)
        self.assertIsNone(mux.accept("manual", twist(0.3), 1.0))
        self.assertIsNotNone(mux.accept("navigation", twist(0.5), 2.0))
        mux.set_mode("MANUAL", 3.0)
        self.assertIsNone(mux.accept("navigation", twist(0.5), 4.0))
        self.assertIsNotNone(mux.accept("manual", twist(0.3), 5.0))

    def test_unknown_mode_falls_back_to_idle_with_zero(self):
        mux = VelocityMux()
        output = mux.set_mode("WEIRD", 0.0)
        self.assertIsNotNone(output)
        self.assertEqual(0.0, output.linear.x)
        self.assertEqual("IDLE", mux.mode)
        self.assertIsNone(mux.accept("navigation", twist(0.5), 1.0))

    def test_explicit_idle_publishes_zero_and_blocks_sources(self):
        mux = VelocityMux()
        mux.set_mode("NAVIGATION", 0.0)
        output = mux.set_mode("IDLE", 1.0)
        self.assertIsNotNone(output)
        self.assertEqual(0.0, output.linear.x)
        self.assertIsNone(mux.accept("navigation", twist(0.5), 2.0))

    def test_unknown_source_raises(self):
        mux = VelocityMux()
        with self.assertRaises(ValueError):
            mux.accept("mystery", twist(0.5), 1.0)

    def test_zero_velocity_message_is_forwarded(self):
        mux = VelocityMux()
        mux.set_mode("NAVIGATION", 0.0)
        forwarded = mux.accept("navigation", twist(0.0), 1.0)
        self.assertIsNotNone(forwarded)
        self.assertEqual(0.0, forwarded.linear.x)


class VelocityMuxFailClosedTests(unittest.TestCase):
    def _navigating(self):
        mux = VelocityMux()
        mux.set_mode("NAVIGATION", 0.0)
        return mux

    def test_nan_publishes_zero_once_then_repeated_invalid_is_ignored(self):
        mux = self._navigating()
        output = mux.accept("navigation", twist(float("nan")), 1.0)
        self.assertIsNotNone(output)
        self.assertEqual(0.0, output.linear.x)
        self.assertIsNone(mux.accept("navigation", twist(float("nan")), 2.0))

    def test_inf_publishes_zero_once(self):
        mux = self._navigating()
        output = mux.accept("navigation", twist(float("inf")), 1.0)
        self.assertIsNotNone(output)
        self.assertEqual(0.0, output.linear.x)

    def test_bounds_violation_publishes_zero_once(self):
        for bad in (twist(1.5), twist(0.5, z=3.0)):
            with self.subTest(bad=bad):
                mux = self._navigating()
                output = mux.accept("navigation", bad, 1.0)
                self.assertIsNotNone(output)
                self.assertEqual(0.0, output.linear.x)

    def test_fresh_valid_message_resumes_after_invalid(self):
        mux = self._navigating()
        mux.accept("navigation", twist(float("nan")), 1.0)
        forwarded = mux.accept("navigation", twist(0.5), 2.0)
        self.assertEqual(0.5, forwarded.linear.x)

    def test_clock_rollback_publishes_zero(self):
        mux = self._navigating()
        output = mux.accept("navigation", twist(0.5), 5.0)
        self.assertEqual(0.5, output.linear.x)
        output = mux.accept("navigation", twist(0.5), 3.0)
        self.assertIsNotNone(output)
        self.assertEqual(0.0, output.linear.x)

    def test_stale_source_publishes_zero(self):
        mux = self._navigating()
        self.assertEqual(0.5, mux.accept("navigation", twist(0.5), 0.0).linear.x)
        self.assertIsNone(mux.tick(0.2))
        output = mux.tick(0.4)
        self.assertIsNotNone(output)
        self.assertEqual(0.0, output.linear.x)
        self.assertIsNone(mux.tick(0.5))

    def test_idle_tick_returns_none(self):
        mux = VelocityMux()
        self.assertIsNone(mux.tick(10.0))


if __name__ == "__main__":
    unittest.main()
