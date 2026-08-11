import math
from decimal import Decimal
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from task_orchestrator.motion_mode import (
    IDLE,
    LINE_FOLLOW,
    NAVIGATION,
    QR_SEARCH,
    STOP_NAVIGATION,
)
from task_orchestrator.velocity_arbiter import VelocityArbiter, validate_positive_finite


def twist(x=0.0, z=0.0):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=x, y=0.0, z=0.0),
        angular=types.SimpleNamespace(x=0.0, y=0.0, z=z),
    )


def values(message):
    return (message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z)


class VelocityArbiterTests(unittest.TestCase):
    def setUp(self):
        self.arbiter = VelocityArbiter(source_timeout=0.3)

    def test_positive_finite_parameter_validation(self):
        for bad in (True, False, 0, -1, math.nan, math.inf, "0.3", Decimal("0.3")):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    validate_positive_finite("value", bad)
        self.assertEqual(0.3, validate_positive_finite("value", 0.3))
        arbiter = VelocityArbiter(0.3)
        self.assertEqual((0.0,) * 6, values(arbiter.set_mode(NAVIGATION, 0.0)))

    def test_values_are_strict_floats_bounded_and_normalized(self):
        arbiter = VelocityArbiter(0.3, max_linear_abs=1.0, max_angular_abs=2.0)
        arbiter.set_mode(NAVIGATION, 1.0)
        valid = twist(1, 2)
        result = arbiter.accept("navigation", valid, 1.1)
        self.assertEqual((1.0, 0.0, 0.0, 0.0, 0.0, 2.0), values(result))
        self.assertTrue(all(type(value) is float for value in values(result)))
        for bad in ("1", True, Decimal("1"), 1e308, math.nan, math.inf):
            with self.subTest(bad=bad):
                arbiter.accept("navigation", twist(0.1), 2.0)
                self.assertEqual((0.0,) * 6, values(arbiter.accept("navigation", twist(bad), 2.1)))
                self.assertIsNone(arbiter.accept("navigation", twist(bad), 2.2))
        arbiter.accept("navigation", twist(0.1), 3.0)
        self.assertEqual((0.0,) * 6, values(arbiter.accept("navigation", twist(1.01), 3.1)))
        arbiter.accept("navigation", twist(0.1), 4.0)
        self.assertEqual((0.0,) * 6, values(arbiter.accept("navigation", twist(0.0, 2.01), 4.1)))

    def test_only_active_source_is_forwarded_and_copy_is_defensive(self):
        self.arbiter.set_mode(NAVIGATION, 1.0)
        self.assertIsNone(self.arbiter.accept("qr", twist(9.0), 1.1))
        incoming = twist(0.75, -0.5)
        output = self.arbiter.accept("navigation", incoming, 1.1)
        incoming.linear.x = 99.0
        self.assertEqual((0.75, 0.0, 0.0, 0.0, 0.0, -0.5), values(output))
        with self.assertRaises(ValueError):
            self.arbiter.accept("avoidance", twist(), 1.2)

    def test_switch_unknown_mode_and_timeout_return_zero_once(self):
        self.assertEqual((0.0,) * 6, values(self.arbiter.set_mode(NAVIGATION, 1.0)))
        self.assertIsNone(self.arbiter.set_mode(NAVIGATION, 1.01))
        self.arbiter.accept("navigation", twist(1.0), 1.1)
        self.assertIsNone(self.arbiter.tick(1.39))
        self.assertEqual((0.0,) * 6, values(self.arbiter.tick(1.41)))
        self.assertIsNone(self.arbiter.tick(2.0))
        self.assertEqual((0.0,) * 6, values(self.arbiter.set_mode(QR_SEARCH, 2.1)))
        self.assertEqual((0.0,) * 6, values(self.arbiter.set_mode("bad", 2.2)))
        self.assertEqual(IDLE, self.arbiter.mode)

    def test_stop_navigation_forwards_only_stop_source(self):
        self.assertEqual((0.0,) * 6, values(self.arbiter.set_mode(STOP_NAVIGATION, 1.0)))
        self.assertIsNone(self.arbiter.accept("navigation", twist(9.0), 1.1))
        self.assertIsNone(self.arbiter.accept("qr", twist(9.0), 1.1))
        output = self.arbiter.accept("stop", twist(0.5, 0.2), 1.2)
        self.assertEqual((0.5, 0.0, 0.0, 0.0, 0.0, 0.2), values(output))

    def test_stop_source_is_ignored_outside_stop_navigation(self):
        self.arbiter.set_mode(NAVIGATION, 1.0)
        self.assertIsNone(self.arbiter.accept("stop", twist(9.0), 1.1))
        self.arbiter.set_mode(IDLE, 2.0)
        self.assertIsNone(self.arbiter.accept("stop", twist(9.0), 2.1))

    def test_transitions_into_and_out_of_stop_navigation_publish_zero(self):
        self.arbiter.set_mode(NAVIGATION, 1.0)
        self.arbiter.accept("navigation", twist(1.0), 1.1)
        into = self.arbiter.set_mode(STOP_NAVIGATION, 1.2)
        self.assertEqual((0.0,) * 6, values(into))
        self.arbiter.accept("stop", twist(0.5), 1.3)
        out = self.arbiter.set_mode(NAVIGATION, 1.4)
        self.assertEqual((0.0,) * 6, values(out))

    def test_stop_source_timeout_and_rollback_fail_closed(self):
        self.arbiter.set_mode(STOP_NAVIGATION, 10.0)
        self.arbiter.accept("stop", twist(0.5), 10.1)
        self.assertIsNone(self.arbiter.tick(10.39))
        self.assertEqual((0.0,) * 6, values(self.arbiter.tick(10.41)))
        self.arbiter.accept("stop", twist(0.5), 10.5)
        self.assertEqual((0.0,) * 6, values(self.arbiter.accept("stop", twist(0.6), 9.9)))

    def test_invalid_values_and_clock_rollback_zero_and_reset(self):
        self.arbiter.set_mode(NAVIGATION, 10.0)
        self.arbiter.accept("navigation", twist(1.0), 10.1)
        self.assertEqual((0.0,) * 6, values(self.arbiter.accept("navigation", twist(math.nan), 10.2)))
        self.assertIsNone(self.arbiter.accept("navigation", twist(math.inf), 10.3))
        self.assertEqual((0.0,) * 6, values(self.arbiter.tick(9.0)))
        self.assertIsNone(self.arbiter.tick(9.1))

    def test_line_follow_forwards_only_line_follow_source(self):
        moving = twist(0.6, 0.3)
        expected_values = values(moving)
        self.assertEqual((0.0,) * 6, values(self.arbiter.set_mode(LINE_FOLLOW, 10.0)))
        self.assertIsNone(self.arbiter.accept("navigation", moving, 10.1))
        self.assertIsNone(self.arbiter.accept("qr", moving, 10.1))
        self.assertIsNone(self.arbiter.accept("stop", moving, 10.1))
        self.assertEqual(expected_values, values(self.arbiter.accept("line_follow", moving, 10.1)))

    def test_line_follow_source_is_ignored_outside_line_follow(self):
        self.arbiter.set_mode(NAVIGATION, 1.0)
        self.assertIsNone(self.arbiter.accept("line_follow", twist(9.0), 1.1))
        self.arbiter.set_mode(IDLE, 2.0)
        self.assertIsNone(self.arbiter.accept("line_follow", twist(9.0), 2.1))

    def test_line_follow_transitions_publish_zero_and_timeout_fails_closed(self):
        self.arbiter.set_mode(NAVIGATION, 10.0)
        self.arbiter.accept("navigation", twist(1.0), 10.1)
        into = self.arbiter.set_mode(LINE_FOLLOW, 10.2)
        self.assertEqual((0.0,) * 6, values(into))
        self.arbiter.accept("line_follow", twist(0.5), 10.3)
        out = self.arbiter.set_mode(NAVIGATION, 10.4)
        self.assertEqual((0.0,) * 6, values(out))
        self.arbiter.set_mode(LINE_FOLLOW, 20.0)
        self.arbiter.accept("line_follow", twist(0.5), 20.1)
        self.assertIsNone(self.arbiter.tick(20.39))
        self.assertEqual((0.0,) * 6, values(self.arbiter.tick(20.41)))
        self.arbiter.accept("line_follow", twist(0.5), 20.5)
        self.assertEqual((0.0,) * 6, values(self.arbiter.accept("line_follow", twist(0.6), 19.9)))


if __name__ == "__main__":
    unittest.main()
