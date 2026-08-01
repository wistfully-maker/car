import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from ucar_waypoint_nav.odom_calibration import AngleAccumulator


class AngleAccumulatorTests(unittest.TestCase):
    def test_accumulates_counterclockwise_across_pi_boundary(self):
        tracker = AngleAccumulator(target=2.0 * math.pi, tolerance=math.radians(1.0))
        tracker.start(math.radians(170.0))

        tracker.update(math.radians(-170.0))

        self.assertAlmostEqual(tracker.accumulated, math.radians(20.0), places=6)

    def test_does_not_finish_before_target_tolerance(self):
        tracker = AngleAccumulator(target=2.0 * math.pi, tolerance=math.radians(1.0))
        tracker.start(0.0)

        tracker.update(math.radians(-2.0))

        self.assertFalse(tracker.done)

    def test_finishes_within_target_tolerance(self):
        tracker = AngleAccumulator(target=2.0 * math.pi, tolerance=math.radians(1.0))
        tracker.start(0.0)
        yaw = 0.0
        for _ in range(36):
            yaw += math.radians(10.0)
            wrapped = math.atan2(math.sin(yaw), math.cos(yaw))
            tracker.update(wrapped)

        self.assertTrue(tracker.done)
        self.assertAlmostEqual(tracker.accumulated, 2.0 * math.pi, places=6)


if __name__ == "__main__":
    unittest.main()
