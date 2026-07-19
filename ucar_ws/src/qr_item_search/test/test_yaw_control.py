import math
import unittest

from qr_item_search.yaw_control import (
    angular_command,
    directed_angular_command,
    normalize_angle,
)


class NormalizeAngleTest(unittest.TestCase):
    def test_normalizes_angle_above_pi(self):
        self.assertAlmostEqual(3.5 - 2 * math.pi, normalize_angle(3.5))

    def test_rejects_non_finite_angle(self):
        for angle in (math.nan, math.inf, -math.inf):
            with self.subTest(angle=angle):
                with self.assertRaises(ValueError):
                    normalize_angle(angle)


class AngularCommandTest(unittest.TestCase):
    def test_uses_shortest_direction_across_angle_boundary(self):
        speed, reached = angular_command(
            math.radians(179),
            math.radians(-179),
            kp=1.0,
            max_speed=1.0,
            tolerance=0.0,
        )

        self.assertAlmostEqual(math.radians(2), speed)
        self.assertFalse(reached)

    def test_returns_zero_within_tolerance_including_boundary(self):
        speed, reached = angular_command(
            0.0,
            0.1,
            kp=2.0,
            max_speed=1.0,
            tolerance=0.1,
        )

        self.assertEqual(0.0, speed)
        self.assertTrue(reached)

    def test_limits_positive_and_negative_commands(self):
        positive, positive_reached = angular_command(
            0.0, 1.0, kp=2.0, max_speed=0.5, tolerance=0.0
        )
        negative, negative_reached = angular_command(
            0.0, -1.0, kp=2.0, max_speed=0.5, tolerance=0.0
        )

        self.assertEqual(0.5, positive)
        self.assertEqual(-0.5, negative)
        self.assertFalse(positive_reached)
        self.assertFalse(negative_reached)

    def test_applies_minimum_effective_speed_outside_tolerance(self):
        positive, positive_reached = angular_command(
            0.0,
            0.05,
            kp=1.2,
            max_speed=0.30,
            tolerance=0.035,
            min_speed=0.11,
        )
        negative, negative_reached = angular_command(
            0.0,
            -0.05,
            kp=1.2,
            max_speed=0.30,
            tolerance=0.035,
            min_speed=0.11,
        )

        self.assertEqual(0.11, positive)
        self.assertEqual(-0.11, negative)
        self.assertFalse(positive_reached)
        self.assertFalse(negative_reached)

    def test_rejects_invalid_control_parameters(self):
        invalid_parameters = (
            {"kp": 0.0, "max_speed": 1.0, "tolerance": 0.0},
            {"kp": 1.0, "max_speed": 0.0, "tolerance": 0.0},
            {"kp": 1.0, "max_speed": 1.0, "tolerance": -0.1},
        )

        for parameters in invalid_parameters:
            with self.subTest(parameters=parameters):
                with self.assertRaises(ValueError):
                    angular_command(0.0, 1.0, **parameters)

    def test_rejects_non_finite_control_parameters(self):
        valid = {
            "current": 0.0,
            "target": 1.0,
            "kp": 1.0,
            "max_speed": 1.0,
            "tolerance": 0.1,
        }

        for name in valid:
            for value in (math.nan, math.inf, -math.inf):
                parameters = dict(valid)
                parameters[name] = value
                with self.subTest(name=name, value=value):
                    with self.assertRaises(ValueError):
                        angular_command(**parameters)


class DirectedAngularCommandTest(unittest.TestCase):
    def test_follows_error_sign_and_limits_magnitude(self):
        self.assertEqual((0.4, False), directed_angular_command(1.0, 0.4, 0.0))
        self.assertEqual((-0.4, False), directed_angular_command(-1.0, 0.4, 0.0))

    def test_applies_minimum_speed_and_inclusive_tolerance(self):
        self.assertEqual((0.2, False), directed_angular_command(0.1, 1.0, 0.05, 0.2))
        self.assertEqual((0.0, True), directed_angular_command(-0.05, 1.0, 0.05))

    def test_rejects_boolean_non_finite_and_invalid_parameters(self):
        valid = {
            "error": 0.1,
            "speed": 1.0,
            "tolerance": 0.0,
            "min_speed": 0.0,
        }
        for name in valid:
            for value in (True, False, math.nan, math.inf, -math.inf):
                parameters = dict(valid)
                parameters[name] = value
                with self.subTest(name=name, value=value):
                    with self.assertRaises(ValueError):
                        directed_angular_command(**parameters)

        for parameters in (
            (0.1, 0.0, 0.0, 0.0),
            (0.1, 1.0, -0.1, 0.0),
            (0.1, 1.0, 0.0, 1.1),
            (0.1, 1.0, 0.0, -0.1),
        ):
            with self.subTest(parameters=parameters):
                with self.assertRaises(ValueError):
                    directed_angular_command(*parameters)


if __name__ == "__main__":
    unittest.main()
