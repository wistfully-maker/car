"""Phase B: lock the vision-limited lidar staging pose estimator.

The estimator converts the vision-confirmed sign bearing into a safe
staging observation pose in the map frame: extract lidar points inside the
vision sector, fit the target front edge deterministically, pick the normal
facing the robot, step back `staging_distance` and transform to map. Every
rejection is a structured deterministic error; NaN / default-origin /
half-finished results are forbidden.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery.staging_pose_estimator import (
    StagingPoseConsistency,
    StagingPoseError,
    StagingPoseEstimator,
    pose_to_quaternion,
    wrap_angle,
)

IDENTITY_TRANSFORM = {"x": 0.0, "y": 0.0, "yaw": 0.0}


def wall_ranges(
    wall_yaw_deg,
    wall_distance,
    sector_center_deg,
    sector_half_deg,
    angle_min_deg=-180.0,
    angle_step_deg=1.0,
    occlude=0.0,
):
    """Build a LaserScan-like range array with one straight wall.

    The wall line is given by n * p = D with unit normal n pointing from
    the robot towards the wall (wall_yaw_deg = direction of n). Points
    outside the vision sector and occluded angles get `occlude` (0.0 or
    NaN) which is invalid for the estimator.
    """
    angles = np.deg2rad(np.arange(angle_min_deg, 180.0, angle_step_deg))
    wall_normal = np.array([
        math.cos(math.radians(wall_yaw_deg)),
        math.sin(math.radians(wall_yaw_deg)),
    ])
    ranges = []
    for angle in angles:
        theta = math.degrees(angle)
        if abs(theta - sector_center_deg) > sector_half_deg:
            ranges.append(occlude)
            continue
        cosine = wall_normal @ np.array([math.cos(angle), math.sin(angle)])
        if cosine <= 1e-6:
            ranges.append(occlude)
            continue
        distance = wall_distance / cosine
        ranges.append(float(distance))
    return ranges, float(angles[0]), float(np.deg2rad(angle_step_deg))


def short_wall_ranges(wall_yaw_deg, wall_distance, extent_deg):
    """A straight wall visible only in a narrow angular window.

    Used to prove the camera/lidar yaw mount offset: only the compensated
    bearing window sees the wall.
    """
    ranges = []
    angles = np.deg2rad(np.arange(-180.0, 180.0, 1.0))
    for angle in angles:
        theta = math.degrees(angle)
        if abs(theta - wall_yaw_deg) <= extent_deg:
            cosine = math.cos(angle - math.radians(wall_yaw_deg))
            ranges.append(wall_distance / cosine if cosine > 1e-6 else 0.0)
        else:
            ranges.append(0.0)
    return ranges, float(angles[0]), float(np.deg2rad(1.0))


def two_segment_wall_ranges(outer_deg, outer_d, inner_deg, inner_d):
    """Two straight wall segments at different distances (poor single-line fit)."""
    ranges = []
    angles = np.deg2rad(np.arange(-90.0, 90.0, 1.0))
    for angle in angles:
        theta = math.degrees(angle)
        if abs(theta - outer_deg) <= 20.0:
            cosine = math.cos(angle - math.radians(outer_deg))
            ranges.append(outer_d / cosine if cosine > 1e-6 else 0.0)
        elif abs(theta - inner_deg) <= 20.0:
            cosine = math.cos(angle - math.radians(inner_deg))
            ranges.append(inner_d / cosine if cosine > 1e-6 else 0.0)
        else:
            ranges.append(0.0)
    return ranges, float(angles[0]), float(np.deg2rad(1.0))


class EstimatorTests(unittest.TestCase):
    def estimate(
        self,
        ranges,
        angle_min,
        angle_increment,
        bearing=0.0,
        half_width=math.radians(30.0),
        transform=IDENTITY_TRANSFORM,
        config=None,
    ):
        return StagingPoseEstimator(config).estimate(
            ranges, angle_min, angle_increment, bearing, half_width, transform
        )

    def assert_raises_reason(self, expected_fragment, *args, **kwargs):
        with self.assertRaises(StagingPoseError) as ctx:
            self.estimate(*args, **kwargs)
        self.assertIn(
            expected_fragment, str(ctx.exception),
            "expected reason containing %r, got %r"
            % (expected_fragment, str(ctx.exception)),
        )

    def test_front_flat_wall_pose(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        result = self.estimate(ranges, angle_min, increment)
        # 表面中心在激光正前方 1m
        self.assertAlmostEqual(1.0, result["surface_center_laser"][0], places=2)
        self.assertAlmostEqual(0.0, result["surface_center_laser"][1], places=2)
        # 法向量朝向机器人（从表面指向原点）
        self.assertLess(result["surface_normal_laser"][0], 0.0)
        self.assertAlmostEqual(-1.0, result["surface_normal_laser"][0], places=2)
        # 观察点退后 0.7m，面向表面中心
        self.assertAlmostEqual(0.3, result["x"], places=2)
        self.assertAlmostEqual(0.0, result["y"], places=2)
        self.assertAlmostEqual(0.0, result["yaw"], places=2)
        self.assertGreaterEqual(result["inlier_count"], 8)
        self.assertLessEqual(result["residual"], 0.04)
        self.assertGreaterEqual(result["confidence"], 0.9)

    def test_map_transform_rotation_and_translation(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        transform = {"x": 1.5, "y": -0.5, "yaw": math.radians(30.0)}
        result = self.estimate(ranges, angle_min, increment, transform=transform)
        # map = R(30°) * laser + (1.5, -0.5)；激光观察点 (0.3, 0)
        expected_x = 0.3 * math.cos(math.radians(30.0)) + 1.5
        expected_y = 0.3 * math.sin(math.radians(30.0)) - 0.5
        self.assertAlmostEqual(expected_x, result["x"], places=2)
        self.assertAlmostEqual(expected_y, result["y"], places=2)
        # map yaw = laser yaw + 30°
        self.assertAlmostEqual(math.radians(30.0), result["yaw"], places=3)

    def test_oblique_wall_normal_faces_robot(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=20.0, wall_distance=1.0,
            sector_center_deg=20.0, sector_half_deg=25.0,
        )
        result = self.estimate(
            ranges, angle_min, increment,
            bearing=math.radians(20.0), half_width=math.radians(25.0),
        )
        nx, ny = result["surface_normal_laser"]
        # 法向量必须朝向机器人：与表面中心方向（从原点到中心）反向
        cx, cy = result["surface_center_laser"]
        self.assertLess(nx * cx + ny * cy, 0.0)
        # 表面中心沿“机器人→墙”方向投影距离约 1.0m
        self.assertAlmostEqual(
            1.0, -(nx * cx + ny * cy), places=1
        )
        # yaw 面向表面中心：yaw = atan2(-n)；斜 20° 墙 → yaw ≈ 20°
        self.assertAlmostEqual(
            math.radians(20.0), result["yaw"], delta=math.radians(2.0)
        )
        # 观察点位于车与表面之间（法向退后）
        step = math.hypot(
            cx - result["x"], cy - result["y"]
        )
        self.assertAlmostEqual(0.7, step, places=2)

    def test_noise_points_filtered_by_range_outlier(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        # 注入两个 3m 杂散点（距离离群）
        ranges[45] = 3.0
        ranges[80] = 3.5
        result = self.estimate(ranges, angle_min, increment)
        self.assertAlmostEqual(0.3, result["x"], places=2)
        self.assertGreaterEqual(result["inlier_count"], 40)

    def test_partially_occluded_sector_still_estimates(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
            occlude=float("nan"),
        )
        # 左半扇区无回波
        for index in range(30, 61):
            ranges[index] = float("nan")
        result = self.estimate(ranges, angle_min, increment)
        self.assertAlmostEqual(0.3, result["x"], places=2)

    def test_camera_lidar_yaw_offset_applied(self):
        # 短墙只出现在 [8°, 20°]。补偿 +5° 后扇区 [-7°, 17°] 覆盖墙并成功；
        # 不补偿时扇区 [-12°, 12°] 只能看到 5 个点，少于 min_valid_points。
        ranges, angle_min, increment = short_wall_ranges(
            wall_yaw_deg=14.0, wall_distance=1.0, extent_deg=6.0,
        )
        result = self.estimate(
            ranges, angle_min, increment,
            bearing=math.radians(0.0),
            half_width=math.radians(10.0),
            config={"camera_lidar_yaw_offset_deg": 5.0},
        )
        self.assertAlmostEqual(
            math.radians(14.0), result["yaw"], delta=math.radians(2.0)
        )
        # 不补偿时扇区覆盖点不足 → 失败
        self.assert_raises_reason(
            "valid points",
            ranges, angle_min, increment,
            math.radians(0.0), math.radians(10.0),
        )

    def test_yaw_normalized_to_pi_range(self):
        # 墙法向接近 180°：yaw 必须落在 [-pi, pi] 且正确回绕
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=179.0, wall_distance=1.0,
            sector_center_deg=179.0, sector_half_deg=15.0,
        )
        result = self.estimate(
            ranges, angle_min, increment,
            bearing=math.radians(179.0), half_width=math.radians(15.0),
        )
        self.assertGreaterEqual(result["yaw"], -math.pi)
        self.assertLess(result["yaw"], math.pi)
        self.assertAlmostEqual(
            result["yaw"],
            wrap_angle(result["yaw"]),
            places=9,
        )

    def test_all_invalid_ranges_fail(self):
        ranges = [float("nan")] * 181
        self.assert_raises_reason(
            "valid points", ranges, -math.pi / 2, math.pi / 180
        )

    def test_zero_ranges_are_invalid(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
            occlude=0.0,
        )
        result = self.estimate(ranges, angle_min, increment)
        self.assertAlmostEqual(0.3, result["x"], places=2)

    def test_out_of_range_distance_fails(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=9.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        self.assert_raises_reason(
            "valid points", ranges, angle_min, increment
        )

    def test_too_few_points_fails(self):
        # 半宽 2° → 扇区内约 5 个点 < min_valid_points=8
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=2.0,
        )
        self.assert_raises_reason(
            "valid points", ranges, angle_min, increment,
            math.radians(0.0), math.radians(2.0),
        )

    def test_fit_residual_too_large_fails(self):
        ranges, angle_min, increment = two_segment_wall_ranges(
            outer_deg=0.0, outer_d=1.0,
            inner_deg=30.0, inner_d=1.3,
        )
        self.assert_raises_reason(
            "residual", ranges, angle_min, increment,
            math.radians(10.0), math.radians(45.0),
        )

    def test_too_few_inliers_fails(self):
        # 31 个共线点完美拟合，但配置 min_inliers=40 → 内点不足
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=15.0,
        )
        self.assert_raises_reason(
            "inliers", ranges, angle_min, increment,
            config={"min_inliers": 40},
        )

    def test_staging_travel_too_close_fails(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        self.assert_raises_reason(
            "too close", ranges, angle_min, increment,
            config={"min_staging_travel": 0.8},
        )

    def test_staging_travel_too_far_fails(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        self.assert_raises_reason(
            "too far", ranges, angle_min, increment,
            config={"max_staging_travel": 0.2},
        )

    def test_surface_closer_than_staging_distance_fails(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=0.5,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        self.assert_raises_reason(
            "staging distance", ranges, angle_min, increment
        )

    def test_output_all_finite(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        result = self.estimate(ranges, angle_min, increment)
        flat = (
            [result["x"], result["y"], result["yaw"]]
            + list(result["surface_center_laser"])
            + list(result["surface_normal_laser"])
            + [result["inlier_count"], result["residual"], result["confidence"]]
        )
        for value in flat:
            self.assertTrue(
                isinstance(value, (int, float)) and math.isfinite(value),
                "non-finite output value: %r" % (value,),
            )

    def test_invalid_transform_rejected(self):
        ranges, angle_min, increment = wall_ranges(
            wall_yaw_deg=0.0, wall_distance=1.0,
            sector_center_deg=0.0, sector_half_deg=30.0,
        )
        with self.assertRaises(StagingPoseError):
            self.estimate(
                ranges, angle_min, increment,
                transform={"x": float("nan"), "y": 0.0, "yaw": 0.0},
            )
        with self.assertRaises(StagingPoseError):
            self.estimate(
                ranges, angle_min, increment,
                transform={"yaw": 0.0},
            )

    def test_pose_to_quaternion_is_normalized(self):
        for yaw in (0.0, 0.7, math.pi, -math.pi + 0.1, 3.0, -2.5):
            q = pose_to_quaternion(yaw)
            self.assertEqual(4, len(q))
            norm = math.sqrt(sum(component * component for component in q))
            self.assertAlmostEqual(1.0, norm, places=6)
            # 绕 z 轴旋转，roll/pitch 为零
            self.assertAlmostEqual(0.0, q[0], places=6)
            self.assertAlmostEqual(0.0, q[1], places=6)


class ConsistencyTests(unittest.TestCase):
    def pose(self, x, y, yaw):
        return {"x": x, "y": y, "yaw": yaw}

    def test_three_consistent_updates_confirm(self):
        check = StagingPoseConsistency()
        self.assertFalse(check.update(self.pose(1.0, 0.5, 0.2)))
        self.assertFalse(check.update(self.pose(1.02, 0.51, 0.21)))
        self.assertTrue(check.update(self.pose(1.01, 0.49, 0.19)))

    def test_inconsistent_pose_restarts_streak(self):
        check = StagingPoseConsistency()
        check.update(self.pose(1.0, 0.5, 0.2))
        check.update(self.pose(1.0, 0.5, 0.2))
        # 偏差超过一致性阈值 → 计数从新参考重新开始
        check.update(self.pose(1.5, 0.5, 0.2))
        self.assertFalse(check.confirmed())
        check.update(self.pose(1.5, 0.5, 0.2))
        self.assertFalse(check.confirmed())
        check.update(self.pose(1.5, 0.5, 0.2))
        self.assertTrue(check.confirmed())

    def test_reset_clears_state(self):
        check = StagingPoseConsistency()
        check.update(self.pose(1.0, 0.5, 0.2))
        check.update(self.pose(1.0, 0.5, 0.2))
        check.reset()
        self.assertFalse(check.update(self.pose(1.0, 0.5, 0.2)))
        self.assertFalse(check.update(self.pose(1.0, 0.5, 0.2)))
        self.assertTrue(check.update(self.pose(1.0, 0.5, 0.2)))

    def test_confirm_threshold_configurable(self):
        check = StagingPoseConsistency({"estimation_confirmations": 2})
        self.assertFalse(check.update(self.pose(1.0, 0.5, 0.2)))
        self.assertTrue(check.update(self.pose(1.0, 0.5, 0.2)))

    def test_invalid_pose_rejected(self):
        check = StagingPoseConsistency()
        self.assertFalse(check.update(None))
        self.assertFalse(check.update({"x": float("nan"), "y": 0.0, "yaw": 0.0}))
        self.assertFalse(check.confirmed())


if __name__ == "__main__":
    unittest.main()
