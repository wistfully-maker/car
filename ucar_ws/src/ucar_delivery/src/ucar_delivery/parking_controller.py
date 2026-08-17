"""Closed-loop white-frame parking controller (ROS-free).

Drives the parking state sequence ALIGN_FRAME -> CENTER_FRAME ->
APPROACH_FRAME -> FINAL_STOP -> VERIFY_STOP with bounded, deadbanded and
rate-limited commands. Line loss, stale camera data, obstacle proximity,
timeouts, cancellation and shutdown all exit through a guaranteed
zero-velocity command before any result.
"""

import math

from ucar_delivery.lidar_safety import LidarSafety


class ParkingController:
    IDLE = "IDLE"
    ALIGN_FRAME = "ALIGN_FRAME"
    CENTER_FRAME = "CENTER_FRAME"
    APPROACH_FRAME = "APPROACH_FRAME"
    FINAL_STOP = "FINAL_STOP"
    VERIFY_STOP = "VERIFY_STOP"
    REACQUIRE = "REACQUIRE"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"

    _ACTIVE_STATES = frozenset(
        (
            ALIGN_FRAME,
            CENTER_FRAME,
            APPROACH_FRAME,
            FINAL_STOP,
            VERIFY_STOP,
            REACQUIRE,
        )
    )
    _TIMEOUT_KEYS = {
        ALIGN_FRAME: "align",
        CENTER_FRAME: "center",
        APPROACH_FRAME: "approach",
        FINAL_STOP: "final_stop",
        VERIFY_STOP: "verify",
    }

    REQUIRED_TIMEOUTS = ("align", "center", "approach", "final_stop", "verify")

    def __init__(self, outputs, clock, timeouts, config):
        self._outputs = outputs
        self._clock = clock
        for key in self.REQUIRED_TIMEOUTS:
            if key not in timeouts:
                raise ValueError("missing parking timeout: %s" % key)
        self._timeouts = dict(timeouts)
        self._k_yaw = float(config["k_yaw"])
        self._k_lateral = float(config["k_lateral"])
        self._max_linear = float(config["max_linear"])
        self._max_angular = float(config["max_angular"])
        self._max_linear_y = float(config.get("max_linear_y", 0.0))
        self._angular_deadband = float(config.get("angular_deadband", 0.0))
        self._center_deadband_px = float(config.get("center_deadband_px", 8.0))
        self._yaw_deadband_px = float(config.get("yaw_deadband_px", 6.0))
        self._min_linear = float(config.get("min_linear", 0.0))
        self._min_angular = float(config.get("min_angular", 0.0))
        self._accel_linear = float(config.get("accel_linear", 0.0))
        self._accel_angular = float(config.get("accel_angular", 0.0))
        self._approach_speed = float(config.get("approach_speed", 0.08))
        self._consecutive_frames = max(1, int(config.get("consecutive_frames", 3)))
        self._max_frame_age = float(config.get("max_frame_age", 0.5))
        capability = config.get("chassis_capability", "differential")
        if capability not in ("differential", "holonomic"):
            raise ValueError("unsupported chassis_capability: %s" % capability)
        self._holonomic = capability == "holonomic"
        self._visual_stop_front_y = float(
            config.get("visual_stop_front_y", 420.0)
        )
        self._verify_duration = float(config.get("verify_duration", 1.0))
        self._velocity_threshold = float(
            config.get("velocity_threshold", 0.02)
        )
        self._frame_reacquire_timeout = float(
            config.get("frame_reacquire_timeout", 1.0)
        )
        self._near_zone_front_y = float(config.get("near_zone_front_y", 380.0))
        self._near_zone_loss_fatal = bool(
            config.get("near_zone_loss_is_fatal", True)
        )
        self._lidar = LidarSafety(
            dict(config, enabled=bool(config.get("lidar_enabled", False)))
        )
        self.state = self.IDLE
        self._deadline = None
        self._last_update = None
        self._last_linear = None
        self._last_angular = None
        self._consecutive = 0
        self._verify_elapsed = 0.0
        self._velocity = (0.0, 0.0, 0.0)
        self._reacquire_from = None

    def _emit(self, action, payload):
        self._outputs.append((action, payload))

    def _now(self):
        return self._clock()

    def start(self):
        if self.state not in (self.IDLE, self.COMPLETE, self.FAILED):
            return
        self.state = self.ALIGN_FRAME
        self._consecutive = 0
        self._verify_elapsed = 0.0
        self._last_linear = None
        self._last_angular = None
        self._deadline = self._now() + self._timeouts[
            self._TIMEOUT_KEYS[self.ALIGN_FRAME]
        ]

    def _transition(self, state):
        self.state = state
        self._consecutive = 0
        self._verify_elapsed = 0.0
        if state == self.COMPLETE:
            self._deadline = None
        else:
            self._deadline = self._now() + self._timeouts[
                self._TIMEOUT_KEYS[state]
            ]

    def _fail(self, message, status=None):
        if self.state in (self.COMPLETE, self.FAILED):
            return
        self.state = self.FAILED
        self._deadline = None
        self._emit("publish_zero", {"reason": message})
        self._emit(
            "controller_result",
            {"status": status or "failed", "message": message},
        )

    def _lose_frame(self, reason, observation=None):
        if self.state == self.REACQUIRE:
            return
        if self.state == self.VERIFY_STOP:
            # 验证阶段丢框：验证必须连续，绝不重获后报告 verified。
            self._fail(reason + " during verification", "frame_lost")
            return
        if self._near_zone_loss_fatal and self._is_near_zone(observation):
            # 规则 8：近墙或近白线时丢框，立即失败，禁止继续盲走。
            self._fail(reason + " in near zone", "frame_lost_near")
            return
        # 规则 7：远区丢框 → 零速并在 frame_reacquire_timeout 内重获
        self._enter_reacquire(reason)

    def _publish_twist(self, linear_x, linear_y, angular_z):
        self._emit(
            "publish_twist",
            {
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z,
            },
        )

    def _rate_limit(self, previous, target, limit, dt):
        if previous is None or limit <= 0.0 or dt <= 0.0:
            return target
        max_step = limit * dt
        delta = target - previous
        if abs(delta) <= max_step:
            return target
        return previous + math.copysign(max_step, delta)

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def _command(self, linear_x, linear_y, angular_z, now):
        if self._last_update is None:
            dt = 0.0
        else:
            dt = now - self._last_update
        angular_z = self._rate_limit(
            self._last_angular, angular_z, self._accel_angular, dt
        )
        linear_x = self._rate_limit(
            self._last_linear, linear_x, self._accel_linear, dt
        )
        self._last_angular = angular_z
        self._last_linear = linear_x
        self._last_update = now
        self._publish_twist(linear_x, linear_y, angular_z)

    def _steering(self, observation):
        yaw_error = observation.far_center_x - observation.near_center_x
        lateral_error = observation.lateral_error
        angular = -self._k_yaw * yaw_error - self._k_lateral * lateral_error
        if abs(angular) < self._angular_deadband:
            angular = 0.0
        return self._clamp(angular, self._max_angular)

    def _aligned(self, observation):
        yaw_error = observation.far_center_x - observation.near_center_x
        return (
            abs(yaw_error) <= self._yaw_deadband_px
            and abs(observation.lateral_error) <= self._center_deadband_px
        )

    def on_odometry_velocity(self, vx, vy, vth, now):
        self._velocity = (vx, vy, vth)

    def update(self, observation, front_range, now):
        if self.state not in self._ACTIVE_STATES:
            return
        if self._lidar.enabled:
            if self._lidar.danger(front_range):
                self._fail(
                    "lidar safety stop at %.3f m" % front_range,
                    "lidar_danger",
                )
                return
            if front_range is None:
                # lidar 启用时无有效扫描绝不能视为安全：零速失败
                self._fail("lidar scan invalid (no valid front range)",
                           "lidar_invalid")
                return
        if self.state == self.REACQUIRE:
            self._update_reacquire(observation, front_range, now)
            return
        if observation is None or not observation.frame_detected:
            self._lose_frame("parking frame lost", observation)
            return
        if now - observation.timestamp > self._max_frame_age:
            self._fail("stale camera data", "frame_stale")
            return
        self._handle_state(observation, front_range, now)

    def _handle_state(self, observation, front_range, now):
        if self.state == self.ALIGN_FRAME:
            self._update_align(observation, now)
        elif self.state == self.CENTER_FRAME:
            self._update_center(observation, now)
        elif self.state == self.APPROACH_FRAME:
            self._update_approach(observation, front_range, now)
        elif self.state == self.FINAL_STOP:
            self._update_final_stop(observation, front_range, now)
        elif self.state == self.VERIFY_STOP:
            self._update_verify(observation, front_range, now)

    def _is_near_zone(self, observation):
        if observation is None:
            return False
        if observation.near_zone_loss:
            return True
        if observation.near_zone:
            return True
        front_y = observation.front_boundary_y
        return front_y is not None and front_y >= self._near_zone_front_y

    def _enter_reacquire(self, reason):
        self._reacquire_from = self.state
        self._consecutive = 0
        self._verify_elapsed = 0.0
        self.state = self.REACQUIRE
        self._deadline = self._now() + self._frame_reacquire_timeout
        self._emit("publish_zero", {"reason": reason})

    def _update_reacquire(self, observation, front_range, now):
        # 重获期间持续零速，禁止任何运动
        self._command(0.0, 0.0, 0.0, now)
        if observation is None or not observation.frame_detected:
            return
        if now - observation.timestamp > self._max_frame_age:
            return
        # 宽限内重获：回到丢框前状态，重新计时并继续处理当前帧
        self.state = self._reacquire_from
        self._consecutive = 0
        self._verify_elapsed = 0.0
        self._deadline = self._now() + self._timeouts[
            self._TIMEOUT_KEYS[self.state]
        ]
        self._handle_state(observation, front_range, now)

    def _update_align(self, observation, now):
        angular = self._steering(observation)
        self._command(0.0, 0.0, angular, now)
        if self._aligned(observation):
            self._consecutive += 1
            if self._consecutive >= self._consecutive_frames:
                self._transition(self.CENTER_FRAME)
        else:
            self._consecutive = 0

    def _update_center(self, observation, now):
        angular = self._steering(observation)
        lateral = 0.0
        if self._holonomic:
            lateral = self._clamp(
                -self._k_lateral * observation.lateral_error,
                self._max_linear_y,
            )
        self._command(0.0, lateral, angular, now)
        if self._aligned(observation):
            self._consecutive += 1
            if self._consecutive >= self._consecutive_frames:
                self._transition(self.APPROACH_FRAME)
        else:
            self._consecutive = 0

    def _approach_target_reached(self, front_range, observation):
        if front_range is not None and front_range <= self._lidar.target_stop_distance:
            return True
        front_y = observation.front_boundary_y
        return front_y is not None and front_y >= self._visual_stop_front_y

    def _update_approach(self, observation, front_range, now):
        angular = self._steering(observation)
        self._command(self._approach_speed, 0.0, angular, now)
        if self._approach_target_reached(front_range, observation):
            self._consecutive += 1
            if self._consecutive >= self._consecutive_frames:
                self._transition(self.FINAL_STOP)
        else:
            self._consecutive = 0

    def _update_final_stop(self, observation, front_range, now):
        self._command(0.0, 0.0, 0.0, now)
        if self._approach_target_reached(front_range, observation):
            self._consecutive += 1
            if self._consecutive >= self._consecutive_frames:
                self._transition(self.VERIFY_STOP)
        else:
            self._consecutive = 0

    def _update_verify(self, observation, front_range, now):
        dt = now - self._last_update if self._last_update is not None else 0.0
        self._command(0.0, 0.0, 0.0, now)
        stopped = all(
            abs(value) <= self._velocity_threshold
            for value in self._velocity
        )
        # 成功同时要求：对齐、白线/雷达终停条件、里程计近零。任一条件
        # 丢失（含终停条件回退）都清零稳定计时，绝不提前 verified。
        if (
            self._aligned(observation)
            and stopped
            and self._approach_target_reached(front_range, observation)
        ):
            self._verify_elapsed += dt
        else:
            self._verify_elapsed = 0.0
        if self._verify_elapsed >= self._verify_duration:
            self._transition(self.COMPLETE)
            self._emit(
                "controller_result", {"status": "ok", "message": ""}
            )

    def tick(self, now):
        if self.state not in self._ACTIVE_STATES or self._deadline is None:
            return
        if now < self._deadline:
            return
        if self.state == self.REACQUIRE:
            self._fail("frame not reacquired within timeout", "frame_lost")
            return
        state = self.state
        self._fail("timeout in state %s" % state, "timed_out")

    def cancel(self, reason):
        if self.state not in self._ACTIVE_STATES:
            return
        self._fail(reason or "cancelled", "cancelled")

    def shutdown(self):
        if self.state == self.IDLE:
            self.state = self.FAILED
            return
        if self.state in (self.COMPLETE, self.FAILED):
            return
        self._fail("controller shutdown", "stopped")
