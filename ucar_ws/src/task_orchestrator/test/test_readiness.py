import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from task_orchestrator.readiness import ReadinessSnapshot, missing_requirements


def healthy(**changes):
    values = dict(
        now=10.0,
        scan_stamp=9.5,
        odom_stamp=9.5,
        map_received=True,
        tf_map_odom=True,
        tf_odom_base=True,
        tf_base_laser=True,
        move_base_available=True,
        lidar_loc_live=True,
        amcl_live=False,
        global_planner="global_planner/GlobalPlanner",
        local_planner="teb_local_planner/TebLocalPlannerROS",
    )
    values.update(changes)
    return ReadinessSnapshot(**values)


class ReadinessTests(unittest.TestCase):
    def test_healthy_snapshot_has_no_missing_requirements(self):
        self.assertEqual([], missing_requirements(healthy(), max_age=2.0))

    def test_reports_every_requirement_in_stable_order(self):
        snapshot = ReadinessSnapshot(now=10.0)
        self.assertEqual(
            [
                "scan not received", "odom not received", "map not received",
                "TF map->odom unavailable", "TF odom->base_link unavailable",
                "TF base_link->laser_frame unavailable",
                "move_base action unavailable", "/lidar_loc is not live",
                "global planner mismatch: expected global_planner/GlobalPlanner, got <unset>",
                "local planner mismatch: expected teb_local_planner/TebLocalPlannerROS, got <unset>",
            ],
            missing_requirements(snapshot, max_age=2.0),
        )

    def test_message_freshness_rejects_old_and_future_stamps(self):
        old = healthy(scan_stamp=7.9, odom_stamp=11.0)
        self.assertEqual(
            ["scan stale (age 2.100s)", "odom timestamp is in the future"],
            missing_requirements(old, max_age=2.0)[:2],
        )

    def test_latched_map_remains_valid_independent_of_age(self):
        self.assertEqual([], missing_requirements(healthy(now=10000.0, scan_stamp=9999.0, odom_stamp=9999.0), 2.0))

    def test_amcl_is_an_explicit_conflict(self):
        self.assertEqual(
            ["/amcl conflicts with /lidar_loc"],
            missing_requirements(healthy(amcl_live=True), max_age=2.0),
        )


if __name__ == "__main__":
    unittest.main()
