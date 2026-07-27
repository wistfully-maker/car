#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "controlled_twist_test.py"
SPEC = importlib.util.spec_from_file_location("controlled_twist_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ControlledTwistTests(unittest.TestCase):
    def test_accepts_low_speed_short_command(self):
        self.assertEqual(
            MODULE.validate_command(0.08, 0.0, 0.0, 1.0),
            (0.08, 0.0, 0.0, 1.0),
        )

    def test_rejects_excessive_translation(self):
        with self.assertRaises(ValueError):
            MODULE.validate_command(0.11, 0.0, 0.0, 1.0)

    def test_rejects_excessive_rotation(self):
        with self.assertRaises(ValueError):
            MODULE.validate_command(0.0, 0.0, 0.26, 1.0)

    def test_rejects_long_or_nonpositive_duration(self):
        for duration in (0.0, -1.0, 2.1):
            with self.subTest(duration=duration):
                with self.assertRaises(ValueError):
                    MODULE.validate_command(0.0, 0.0, 0.1, duration)

    def test_rejects_combined_axes(self):
        with self.assertRaises(ValueError):
            MODULE.validate_command(0.05, 0.05, 0.0, 1.0)

    def test_publish_cycles_cover_full_requested_duration(self):
        self.assertEqual(MODULE.publish_cycles(2.0, 20), 40)
        self.assertEqual(MODULE.publish_cycles(0.1, 20), 2)


if __name__ == "__main__":
    unittest.main()
