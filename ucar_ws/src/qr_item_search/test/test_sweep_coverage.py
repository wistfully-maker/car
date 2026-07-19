import math
import unittest
from unittest.mock import patch

from qr_item_search.sweep_coverage import CoverageMap, YawTracker


class YawTrackerTest(unittest.TestCase):
    def test_normalizes_each_delta_with_shared_yaw_control_helper(self):
        tracker = YawTracker()
        tracker.reset(1.0)

        with patch("qr_item_search.sweep_coverage.normalize_angle", return_value=0.25) as normalize:
            self.assertEqual(0.25, tracker.update(1.5))

        normalize.assert_called_once_with(0.5)

    def test_accumulates_forward_motion_across_pi(self):
        tracker = YawTracker()
        tracker.reset(math.radians(179))

        self.assertAlmostEqual(math.radians(2), tracker.update(math.radians(-179)))
        self.assertAlmostEqual(math.radians(3), tracker.update(math.radians(-178)))
        self.assertAlmostEqual(math.radians(3), tracker.accumulated)

    def test_accumulates_reverse_motion_across_minus_pi(self):
        tracker = YawTracker()
        tracker.reset(math.radians(-179))

        self.assertAlmostEqual(math.radians(-2), tracker.update(math.radians(179)))
        self.assertAlmostEqual(math.radians(-3), tracker.update(math.radians(178)))

    def test_first_update_resets_tracker_and_accumulated_is_read_only(self):
        tracker = YawTracker()

        self.assertEqual(0.0, tracker.update(1.25))
        self.assertEqual(0.0, tracker.accumulated)
        with self.assertRaises(AttributeError):
            tracker.accumulated = 1.0

    def test_rejects_boolean_and_non_finite_yaw(self):
        tracker = YawTracker()
        for value in (True, False, math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    tracker.reset(value)
                with self.assertRaises(ValueError):
                    tracker.update(value)


class CoverageMapTest(unittest.TestCase):
    def test_uses_specified_default_parameters(self):
        coverage = CoverageMap()

        self.assertEqual(24, coverage.sector_count)
        self.assertEqual(2, coverage.minimum_frames)
        self.assertEqual(0.25, coverage.overexposed_threshold)
        self.assertEqual(30.0, coverage.sharpness_threshold)
        self.assertAlmostEqual(math.radians(10.0), coverage.margin)

    def test_maps_two_pi_and_small_positive_angle_to_sector_zero(self):
        coverage = CoverageMap()
        coverage.record(2 * math.pi, 50.0, 0.0, 50.0, False)
        coverage.record(2 * math.pi + 0.001, 60.0, 0.0, 60.0, False)

        self.assertEqual(2, coverage.sectors[0].frame_count)
        self.assertEqual(0, coverage.sectors[1].frame_count)

    def test_accumulates_sector_statistics_and_decoded_state(self):
        coverage = CoverageMap()
        coverage.record(0.1, 20.0, 0.2, 10.0, False)
        coverage.record(0.1, 30.0, 0.3, 40.0, True)
        sector = coverage.sectors[0]

        self.assertEqual(2, sector.frame_count)
        self.assertEqual(50.0, sector.brightness_sum)
        self.assertEqual(0.5, sector.overexposed_sum)
        self.assertEqual(50.0, sector.sharpness_sum)
        self.assertTrue(sector.decoded)

    def test_merges_adjacent_low_quality_sectors(self):
        coverage = CoverageMap(sector_count=4, margin=0.0)
        for index in range(4):
            for _ in range(2):
                coverage.record(index * math.pi / 2 + 0.1, 50.0, 0.5 if index < 2 else 0.0, 50.0, False)
        intervals = coverage.rescan_intervals()

        self.assertEqual(1, len(intervals))
        self.assertEqual(1, intervals[0].priority)
        self.assertAlmostEqual(0.0, intervals[0].start)
        self.assertAlmostEqual(math.pi, intervals[0].end)

    def test_merges_last_and_first_sector_across_zero(self):
        coverage = CoverageMap(sector_count=4, margin=0.0)
        for index in range(4):
            for _ in range(2):
                coverage.record(index * math.pi / 2 + 0.1, 50.0, 0.5 if index in (0, 3) else 0.0, 50.0, False)
        intervals = coverage.rescan_intervals()

        quality = [interval for interval in intervals if interval.priority == 1]
        self.assertEqual(1, len(quality))
        self.assertAlmostEqual(3 * math.pi / 2, quality[0].start)
        self.assertAlmostEqual(5 * math.pi / 2, quality[0].end)

    def test_cross_zero_merge_extends_by_default_margin(self):
        coverage = CoverageMap(sector_count=4)
        for index in range(4):
            for _ in range(2):
                coverage.record(index * math.pi / 2 + 0.1, 50.0, 0.5 if index in (0, 3) else 0.0, 50.0, False)

        interval = coverage.rescan_intervals()[0]
        self.assertLess(interval.start, 3 * math.pi / 2)
        self.assertGreater(interval.end, 5 * math.pi / 2)

    def test_prioritizes_uncovered_then_quality_then_decoded_neighbors_then_start(self):
        coverage = CoverageMap(sector_count=8, margin=0.0)
        for index in range(8):
            for _ in range(2):
                coverage.record(
                    index * math.tau / 8 + 0.01,
                    50.0,
                    0.5 if index == 1 else 0.0,
                    50.0,
                    index == 4,
                )
        intervals = coverage.rescan_intervals()
        self.assertEqual([(0, 2, 1), (3, 4, 2), (5, 6, 2)], [
            (round(interval.start / (math.tau / 8)), round(interval.end / (math.tau / 8)), interval.priority)
            for interval in intervals
        ])

    def test_merges_adjacent_candidates_with_different_priorities(self):
        coverage = CoverageMap(sector_count=4, margin=0.0)
        for index in range(4):
            for _ in range(2):
                coverage.record(index * math.pi / 2 + 0.1, 50.0, 0.5 if index == 1 else 0.0, 50.0, False)

        intervals = coverage.rescan_intervals()
        self.assertEqual(1, len(intervals))
        self.assertEqual((0.0, math.pi, 1), (intervals[0].start, intervals[0].end, intervals[0].priority))

    def test_merges_cross_zero_candidates_with_different_priorities(self):
        coverage = CoverageMap(sector_count=24, margin=0.0)
        for index in range(23):
            for _ in range(2):
                coverage.record(index * math.tau / 24 + 0.01, 50.0, 0.0, 50.0, False)

        interval = coverage.rescan_intervals()[0]
        self.assertAlmostEqual(23 * math.tau / 24, interval.start)
        self.assertAlmostEqual(25 * math.tau / 24, interval.end)
        self.assertEqual(0, interval.priority)

    def test_overexposure_at_threshold_requires_rescan(self):
        coverage = CoverageMap(sector_count=1, margin=0.0)
        for _ in range(2):
            coverage.record(0.0, 50.0, 0.25, 50.0, False)

        interval = coverage.rescan_intervals()[0]
        self.assertEqual(1, interval.priority)

    def test_uncovered_has_priority_over_other_reasons_and_healthy_map_has_start_fallback(self):
        coverage = CoverageMap(sector_count=4, margin=0.0)
        coverage.record(0.1, 50.0, 0.0, 50.0, True)
        intervals = coverage.rescan_intervals()
        self.assertEqual(0, intervals[0].priority)

        healthy = CoverageMap(sector_count=4, margin=0.0)
        for index in range(4):
            for _ in range(2):
                healthy.record(index * math.tau / 4 + 0.1, 50.0, 0.0, 50.0, False)
        intervals = healthy.rescan_intervals()
        self.assertEqual(1, len(intervals))
        self.assertEqual(3, intervals[0].priority)

    def test_rejects_invalid_parameters_and_records(self):
        for arguments in (
            {"sector_count": 0},
            {"sector_count": True},
            {"minimum_frames": 0},
            {"overexposed_threshold": 1.1},
            {"sharpness_threshold": math.nan},
            {"margin": -0.1},
        ):
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    CoverageMap(**arguments)

        coverage = CoverageMap()
        coverage.record(0.0, 1.0, 0.0, 1.0, False)
        coverage.record(0.0, 1.0, 1.0, 1.0, False)
        invalid_records = (
            (math.nan, 1.0, 0.0, 1.0, False),
            (0.0, True, 0.0, 1.0, False),
            (0.0, 1.0, 1.1, 1.0, False),
            (0.0, 1.0, -0.1, 1.0, False),
            (0.0, 1.0, 0.0, math.inf, False),
            (0.0, 1.0, 0.0, 1.0, 1),
        )
        for record in invalid_records:
            with self.subTest(record=record):
                with self.assertRaises(ValueError):
                    coverage.record(*record)


if __name__ == "__main__":
    unittest.main()
