import math
import unittest

from qr_item_search.yaw_control import angular_command, normalize_angle


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


if __name__ == "__main__":
    unittest.main()
