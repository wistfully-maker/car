"""Vision-limited lidar staging pose estimator (ROS-free).

Turns a vision-confirmed sign bearing and half-width into a safe staging
observation pose in the map frame:

1. apply the camera/lidar yaw mount offset to the detection bearing;
2. extract the finite, in-range lidar points inside the vision sector;
3. drop range outliers so a stray far/close point cannot skew the fit;
4. fit the target front edge with PCA and pick the normal facing the robot;
5. step back `staging_distance` from the surface center along that normal
   to get the observation pose;
6. transform the pose to the map frame with the verified transform.

Every rejection raises StagingPoseError with a deterministic reason. NaN,
default-origin and half-finished results are forbidden: the estimated pose
is only a safe viewpoint from which the white frame must be re-acquired; it
is never the final parking point.
"""

import math

import numpy as np

DEFAULT_CONFIG = {
    "camera_lidar_yaw_offset_deg": 0.0,
    "bearing_margin_deg": 2.0,
    "min_valid_points": 8,
    "min_inliers": 6,
    "max_fit_residual": 0.04,
    "max_range_spread": 0.50,
    "staging_distance": 0.70,
    "min_staging_travel": 0.15,
    "max_staging_travel": 2.00,
    "min_range": 0.05,
    "max_range": 8.0,
}


class StagingPoseError(ValueError):
    """Deterministic, structured rejection of a staging pose estimate."""


def wrap_angle(angle):
    """Wrap an angle into [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_to_quaternion(yaw):
    """Unit quaternion (x, y, z, w) for a pure yaw rotation."""
    half = yaw / 2.0
    q = (
        0.0,
        0.0,
        math.sin(half),
        math.cos(half),
    )
    norm = math.sqrt(sum(component * component for component in q))
    return tuple(component / norm for component in q)


class StagingPoseEstimator:
    def __init__(self, config=None):
        merged = dict(DEFAULT_CONFIG)
        merged.update(config or {})
        self._config = merged

    def estimate(
        self,
        ranges,
        angle_min,
        angle_increment,
        detection_bearing_rad,
        detection_half_width_rad,
        laser_to_map_transform,
    ):
        config = self._config
        offset = math.radians(float(config["camera_lidar_yaw_offset_deg"]))
        margin = math.radians(float(config["bearing_margin_deg"]))
        bearing = float(detection_bearing_rad) + offset
        half = float(detection_half_width_rad) + margin
        min_range = float(config["min_range"])
        max_range = float(config["max_range"])

        points = self._extract_points(
            ranges, angle_min, angle_increment, bearing, half,
            min_range, max_range,
        )
        filtered = self._filter_range_outliers(
            points, float(config["max_range_spread"]),
            int(config["min_valid_points"]),
        )
        center, line_dir = self._fit_line(filtered)
        normal = self._normal_facing_robot(line_dir, center)
        residuals = self._point_residuals(filtered, center, line_dir)
        residual = float(math.sqrt(np.mean(residuals ** 2)))
        inliers = int((residuals <= float(config["max_fit_residual"])).sum())

        self._check_fit(
            residual, inliers, config, center, float(config["staging_distance"])
        )

        surface_center = (float(center[0]), float(center[1]))
        surface_normal = (float(normal[0]), float(normal[1]))
        observation = center + normal * float(config["staging_distance"])
        travel = float(np.linalg.norm(observation))
        if travel < float(config["min_staging_travel"]):
            raise StagingPoseError(
                "staging pose too close (travel %.3f m)" % travel
            )
        if travel > float(config["max_staging_travel"]):
            raise StagingPoseError(
                "staging pose too far (travel %.3f m)" % travel
            )

        transform = self._require_transform(laser_to_map_transform)
        map_pose = self._to_map(observation, transform)
        yaw_laser = math.atan2(-normal[1], -normal[0])
        yaw_map = wrap_angle(yaw_laser + transform["yaw"])

        result = {
            "x": map_pose[0],
            "y": map_pose[1],
            "yaw": yaw_map,
            "surface_center_laser": surface_center,
            "surface_normal_laser": surface_normal,
            "inlier_count": inliers,
            "residual": residual,
            "confidence": inliers / len(filtered),
        }
        self._require_finite_result(result)
        return result

    def _extract_points(
        self, ranges, angle_min, angle_increment, bearing, half,
        min_range, max_range,
    ):
        points = []
        if ranges is None:
            return points
        for index, value in enumerate(ranges):
            angle = float(angle_min) + index * float(angle_increment)
            if abs(wrap_angle(angle - bearing)) > half:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                continue
            distance = float(value)
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < min_range or distance > max_range:
                continue
            points.append(
                (distance * math.cos(angle), distance * math.sin(angle))
            )
        if len(points) < int(self._config["min_valid_points"]):
            raise StagingPoseError(
                "not enough valid points in vision sector (%d)"
                % len(points)
            )
        return points

    def _filter_range_outliers(self, points, max_spread, minimum):
        distances = [math.hypot(x, y) for x, y in points]
        median = float(np.median(distances))
        filtered = [
            point
            for point, distance in zip(points, distances)
            if abs(distance - median) <= max_spread
        ]
        if len(filtered) < minimum:
            raise StagingPoseError(
                "too few points after range outlier filtering (%d)"
                % len(filtered)
            )
        return filtered

    def _fit_line(self, points):
        array = np.asarray(points, dtype=float)
        center = array.mean(axis=0)
        centered = array - center
        covariance = centered.T @ centered / len(array)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        line_dir = eigenvectors[:, int(np.argmax(eigenvalues))]
        return center, line_dir

    def _normal_facing_robot(self, line_dir, center):
        # 前缘是直线：法向量垂直于线方向；取朝向机器人一侧（与表面中心
        # 方向反向），不能随机翻转。
        normal = np.array([-line_dir[1], line_dir[0]])
        if float(normal @ center) > 0.0:
            normal = -normal
        return normal

    def _point_residuals(self, points, center, line_dir):
        array = np.asarray(points, dtype=float) - center
        perpendicular = np.array([-line_dir[1], line_dir[0]])
        return np.abs(array @ perpendicular)

    def _check_fit(self, residual, inliers, config, center, staging_distance):
        if residual > float(config["max_fit_residual"]):
            raise StagingPoseError(
                "fit residual too large (%.4f m)" % residual
            )
        if inliers < int(config["min_inliers"]):
            raise StagingPoseError(
                "not enough inliers in fit (%d)" % inliers
            )
        surface_distance = float(np.linalg.norm(center))
        if surface_distance < staging_distance:
            raise StagingPoseError(
                "surface too close for staging distance "
                "(%.3f m < %.3f m)" % (surface_distance, staging_distance)
            )

    def _require_transform(self, transform):
        if not isinstance(transform, dict):
            raise StagingPoseError("laser to map transform must be a dict")
        try:
            values = (
                float(transform["x"]),
                float(transform["y"]),
                float(transform["yaw"]),
            )
        except (KeyError, TypeError, ValueError):
            raise StagingPoseError(
                "laser to map transform must carry finite x, y, yaw"
            )
        if not all(math.isfinite(value) for value in values):
            raise StagingPoseError(
                "laser to map transform must carry finite x, y, yaw"
            )
        return {"x": values[0], "y": values[1], "yaw": values[2]}

    def _to_map(self, point, transform):
        cosine = math.cos(transform["yaw"])
        sine = math.sin(transform["yaw"])
        return (
            cosine * point[0] - sine * point[1] + transform["x"],
            sine * point[0] + cosine * point[1] + transform["y"],
        )

    def _require_finite_result(self, result):
        flat = (
            [result["x"], result["y"], result["yaw"]]
            + list(result["surface_center_laser"])
            + list(result["surface_normal_laser"])
            + [result["inlier_count"], result["residual"],
               result["confidence"]]
        )
        for value in flat:
            if not math.isfinite(value):
                raise StagingPoseError(
                    "non-finite staging pose output: %r" % (value,)
                )


class StagingPoseConsistency:
    """Consecutive-estimate confirmation gate (ROS-free).

    The estimator result is only trusted once `estimation_confirmations`
    consecutive estimates agree within the xy/yaw thresholds. Any estimate
    outside the thresholds restarts the streak against the new reference.
    """

    DEFAULT_CONFIG = {
        "estimation_confirmations": 3,
        "estimation_consistency_xy": 0.10,
        "estimation_consistency_yaw_deg": 8.0,
    }

    def __init__(self, config=None):
        merged = dict(self.DEFAULT_CONFIG)
        merged.update(config or {})
        self._need = max(1, int(merged["estimation_confirmations"]))
        self._xy = float(merged["estimation_consistency_xy"])
        self._yaw = math.radians(float(merged["estimation_consistency_yaw_deg"]))
        self._streak = 0
        self._reference = None

    def reset(self):
        self._streak = 0
        self._reference = None

    def _valid(self, pose):
        if not isinstance(pose, dict):
            return False
        try:
            values = (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))
        except (KeyError, TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values)

    def update(self, pose):
        """Feed one estimate; returns True when the streak is confirmed."""
        if not self._valid(pose):
            self.reset()
            return False
        if self._reference is None:
            self._reference = dict(pose)
            self._streak = 1
            return self._streak >= self._need
        reference = self._reference
        close_xy = math.hypot(
            float(pose["x"]) - reference["x"],
            float(pose["y"]) - reference["y"],
        ) <= self._xy
        close_yaw = abs(wrap_angle(
            float(pose["yaw"]) - reference["yaw"]
        )) <= self._yaw
        if close_xy and close_yaw:
            self._streak += 1
        else:
            self._reference = dict(pose)
            self._streak = 1
        return self._streak >= self._need

    def confirmed(self):
        return self._streak >= self._need
