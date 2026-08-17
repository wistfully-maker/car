#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
独立坡道干预节点（零侵入 + 自动恢复导航）
- 不修改原有 Action_nav.py
- 监听 /move_base_simple/goal 缓存最新目标
- 检测到坡道后，取消 move_base 目标，盲开穿越
- 穿越完成后清除 costmap，自动重新发布目标，恢复导航
- 不依赖 AMCL，完全兼容 lidar_loc

使用方式：
    直接运行即可，不需要额外启动底层驱动（总命令已提供）
    rosrun ucar_avoid slope_intervention.py
"""

import math
import threading
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

# ==================== 可调参数 ====================
IMU_TOPIC = "/imu"
SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
GOAL_TOPIC = "/move_base_simple/goal"

# 俯仰角检测
PITCH_RAMP_THRESH = 5.0      # 度
PITCH_RAMP_TICKS = 3         # 连续帧数（0.6s）

# 穿越速度
SPEED_FLAT = 0.20
SPEED_UP = 0.30
SPEED_PLATFORM = 0.20
SPEED_DOWN = 0.12

# 俯仰角状态阈值
PITCH_UP_THRESH = 5.0
PITCH_DOWN_THRESH = -5.0
PITCH_FLAT_THRESH = 3.0
PITCH_STABLE_FRAMES = 5

# 穿越保护
RAMP_MAX_TIME = 15.0
DIST_HARD_CEILING = 3.5
MIN_DOWN_DIST = 0.30

# 偏航保持
YAW_HOLD_KP = 0.02
YAW_HOLD_KP_CLIMB = 0.12
YAW_HOLD_LIMIT = 0.3
YAW_LIMIT_DEG = 12.0

# 护栏区矫正（若不需要可将 GUARDRAIL_MIN_POINTS 设为 999）
GUARDRAIL_LEFT_ANG = (70.0, 110.0)
GUARDRAIL_RIGHT_ANG = (250.0, 290.0)
GUARDRAIL_TARGET_LEFT = 0.225
GUARDRAIL_TARGET_RIGHT = 0.675
GUARDRAIL_ZONE_DIST = 0.60
GUARDRAIL_CENTERLINE_KP = 0.08
GUARDRAIL_CENTERLINE_MAX = 0.25
GUARDRAIL_MIN_POINTS = 5
GUARDRAIL_LOSS_FRAMES = 10


class SlopeIntervention:
    def __init__(self):
        rospy.init_node('slope_intervention', anonymous=True)

        # ---- 发布器 ----
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=1)

        # ---- move_base action 客户端 ----
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0))

        # ---- clear_costmaps 服务 ----
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
            self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        except:
            self.clear_costmaps_srv = None
            rospy.logwarn("[slope] clear_costmaps 服务不可用")

        # ---- 目标缓存 ----
        self.last_goal = None
        self.goal_received = False

        # ---- 状态变量 ----
        self.pitch_baseline = 0.0
        self.pitch_samples = []
        self.pitch_baseline_ready = False
        self.imu_pitch_deg = 0.0

        self.odom_yaw = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_dist = 0.0
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.odom_initialized = False

        self.laser_scan = None

        self.ramp_in_progress = False
        self.ramp_handled = False
        self.pitch_streak = 0

        # ---- 订阅 ----
        rospy.Subscriber(IMU_TOPIC, Imu, self.imu_callback)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_callback)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.goal_callback)

        # ---- 看门狗定时器 ----
        rospy.Timer(rospy.Duration(0.2), self.watchdog_tick)

        rospy.loginfo("=" * 50)
        rospy.loginfo("[slope] 独立坡道干预节点已启动")
        rospy.loginfo("[slope] 等待 IMU 基准标定...")
        rospy.loginfo("[slope] 检测到坡道后将自动穿越并恢复导航")
        rospy.loginfo("=" * 50)

    # ---------- 目标缓存 ----------
    def goal_callback(self, msg):
        self.last_goal = msg
        self.goal_received = True
        rospy.loginfo_throttle(2.0, "[slope] 已缓存目标: (%.2f, %.2f)",
                               msg.pose.position.x, msg.pose.position.y)

    # ---------- IMU 回调 ----------
    def imu_callback(self, msg):
        q = msg.orientation
        _, pitch_rad, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_pitch_deg = math.degrees(pitch_rad)

        if not self.pitch_baseline_ready:
            self.pitch_samples.append(self.imu_pitch_deg)
            if len(self.pitch_samples) >= 20:
                self.pitch_baseline = sorted(self.pitch_samples)[len(self.pitch_samples)//2]
                self.pitch_baseline_ready = True
                rospy.loginfo("[slope] ✅ 俯仰角基准标定完成: %.2f°", self.pitch_baseline)

    # ---------- 里程计回调 ----------
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_yaw = math.degrees(yaw)

        if not self.odom_initialized:
            self.odom_initialized = True
            self.last_odom_x = self.odom_x
            self.last_odom_y = self.odom_y
            return

        dx = self.odom_x - self.last_odom_x
        dy = self.odom_y - self.last_odom_y
        self.odom_dist += math.sqrt(dx*dx + dy*dy)
        self.last_odom_x = self.odom_x
        self.last_odom_y = self.odom_y

    # ---------- 激光雷达回调 ----------
    def scan_callback(self, msg):
        self.laser_scan = msg

    # ---------- 看门狗 ----------
    def watchdog_tick(self, event):
        if not self.pitch_baseline_ready or self.ramp_handled or self.ramp_in_progress:
            return
        if not self.goal_received:
            return

        pitch_dev = abs(self.imu_pitch_deg - self.pitch_baseline)
        if pitch_dev > PITCH_RAMP_THRESH:
            self.pitch_streak += 1
            if self.pitch_streak >= PITCH_RAMP_TICKS:
                rospy.logwarn("[slope] 🔔 检测到坡道！俯仰角偏差 %.1f°", pitch_dev)
                self.pitch_streak = 0
                threading.Thread(target=self.start_intervention, daemon=True).start()
        else:
            self.pitch_streak = 0

    # ---------- 护栏区辅助 ----------
    def get_guardrail_distances(self):
        if self.laser_scan is None:
            return 0.0, 0.0, False
        try:
            ranges = self.laser_scan.ranges
            n = len(ranges)
            angle_min = self.laser_scan.angle_min
            angle_inc = self.laser_scan.angle_increment
            range_min = self.laser_scan.range_min
            range_max = self.laser_scan.range_max

            def _collect(start_deg, end_deg):
                start = int(round((math.radians(start_deg) - angle_min) / angle_inc))
                end = int(round((math.radians(end_deg) - angle_min) / angle_inc))
                vals = []
                for i in range(max(0, start), min(n, end+1)):
                    r = ranges[i]
                    if not math.isinf(r) and not math.isnan(r) and range_min < r < range_max:
                        vals.append(r)
                return vals

            left_vals = _collect(GUARDRAIL_LEFT_ANG[0], GUARDRAIL_LEFT_ANG[1])
            right_vals = _collect(GUARDRAIL_RIGHT_ANG[0], GUARDRAIL_RIGHT_ANG[1])
            if len(left_vals) < GUARDRAIL_MIN_POINTS or len(right_vals) < GUARDRAIL_MIN_POINTS:
                return 0.0, 0.0, False
            left_med = sorted(left_vals)[len(left_vals)//2]
            right_med = sorted(right_vals)[len(right_vals)//2]
            return left_med, right_med, True
        except:
            return 0.0, 0.0, False

    # ---------- 穿越执行 ----------
    def execute_ramp_crossing(self):
        rospy.loginfo("[slope] 开始穿越坡道...")

        self.odom_dist = 0.0
        self.last_odom_x = self.odom_x
        self.last_odom_y = self.odom_y

        start_time = rospy.Time.now()
        initial_yaw = self.odom_yaw

        guardrail_locked_yaw = initial_yaw
        guardrail_seen = False
        guardrail_loss_count = 0
        in_guardrail_zone = False
        guardrail_zone_start_dist = 0.0
        guardrail_zone_done = False

        state = "FLAT_BEFORE"
        stable_count = 0
        dist_at_up = 0.0
        dist_at_platform = 0.0
        dist_at_down = 0.0

        cmd = Twist()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            pitch = self.imu_pitch_deg - self.pitch_baseline
            dist = self.odom_dist
            elapsed = (rospy.Time.now() - start_time).to_sec()

            yaw_err = self.odom_yaw - initial_yaw
            if yaw_err > 180.0: yaw_err -= 360.0
            elif yaw_err < -180.0: yaw_err += 360.0

            if abs(yaw_err) > YAW_LIMIT_DEG:
                rospy.logwarn("[slope] ❌ 偏航超限 %.1f°", yaw_err)
                break
            if dist > DIST_HARD_CEILING:
                rospy.logwarn("[slope] ❌ 距离超限 %.2fm", dist)
                break
            if elapsed > RAMP_MAX_TIME:
                rospy.logwarn("[slope] ❌ 穿越超时 %.1fs", elapsed)
                break

            # 护栏区
            ang_z_center = 0.0
            if state == "FLAT_BEFORE" and not guardrail_zone_done:
                if not in_guardrail_zone and pitch > PITCH_FLAT_THRESH:
                    in_guardrail_zone = True
                    guardrail_zone_start_dist = dist
                    rospy.loginfo("[slope] 护栏区激活")
                if in_guardrail_zone:
                    left, right, valid = self.get_guardrail_distances()
                    if valid:
                        guardrail_loss_count = 0
                        guardrail_seen = True
                        guardrail_locked_yaw = self.odom_yaw
                        left_err = GUARDRAIL_TARGET_LEFT - left
                        right_err = right - GUARDRAIL_TARGET_RIGHT
                        center_err = (left_err + right_err) / 2.0
                        ang_z_center = -GUARDRAIL_CENTERLINE_KP * center_err
                        ang_z_center = max(-GUARDRAIL_CENTERLINE_MAX,
                                           min(GUARDRAIL_CENTERLINE_MAX, ang_z_center))
                    else:
                        guardrail_loss_count += 1
                    zone_dist = dist - guardrail_zone_start_dist
                    if guardrail_loss_count >= GUARDRAIL_LOSS_FRAMES or zone_dist > GUARDRAIL_ZONE_DIST:
                        in_guardrail_zone = False
                        guardrail_zone_done = True
                        rospy.loginfo("[slope] 护栏区退出")

            # 速度状态机
            if state == "FLAT_BEFORE":
                cmd.linear.x = SPEED_FLAT
                if pitch > PITCH_UP_THRESH:
                    state = "CLIMBING_UP"
                    dist_at_up = dist
                    rospy.loginfo("[slope] ⬆ CLIMBING_UP (pitch=%.1f°)", pitch)
            elif state == "CLIMBING_UP":
                cmd.linear.x = SPEED_UP
                if abs(pitch) < PITCH_FLAT_THRESH:
                    stable_count += 1
                    if stable_count >= PITCH_STABLE_FRAMES:
                        state = "ON_PLATFORM"
                        stable_count = 0
                        dist_at_platform = dist
                        rospy.loginfo("[slope] ➡ ON_PLATFORM")
                else:
                    stable_count = 0
            elif state == "ON_PLATFORM":
                cmd.linear.x = SPEED_PLATFORM
                if pitch < PITCH_DOWN_THRESH:
                    state = "CLIMBING_DOWN"
                    dist_at_down = dist
                    rospy.loginfo("[slope] ⬇ CLIMBING_DOWN")
            elif state == "CLIMBING_DOWN":
                cmd.linear.x = SPEED_DOWN
                if abs(pitch) < PITCH_FLAT_THRESH:
                    stable_count += 1
                    down_dist = dist - dist_at_down
                    if stable_count >= PITCH_STABLE_FRAMES and down_dist >= MIN_DOWN_DIST:
                        state = "DONE"
                        rospy.loginfo("[slope] ✅ DONE (下坡完成)")
                        break
                else:
                    stable_count = 0

            # 偏航控制
            climb_states = ("CLIMBING_UP", "ON_PLATFORM", "CLIMBING_DOWN")
            yaw_kp = YAW_HOLD_KP_CLIMB if state in climb_states else YAW_HOLD_KP

            if in_guardrail_zone and guardrail_seen:
                yaw_hold = yaw_kp * yaw_err
                yaw_hold = max(-YAW_HOLD_LIMIT, min(YAW_HOLD_LIMIT, yaw_hold))
                ang_z = ang_z_center * 0.7 + yaw_hold * 0.3
            else:
                yaw_ref = guardrail_locked_yaw if guardrail_seen else initial_yaw
                yaw_err_ref = self.odom_yaw - yaw_ref
                if yaw_err_ref > 180.0: yaw_err_ref -= 360.0
                elif yaw_err_ref < -180.0: yaw_err_ref += 360.0
                ang_z = yaw_kp * yaw_err_ref

            cmd.angular.z = max(-YAW_HOLD_LIMIT, min(YAW_HOLD_LIMIT, ang_z))

            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        success = (state == "DONE")
        rospy.loginfo("[slope] 穿越结果: %s", "✅ 成功" if success else "❌ 失败")
        return success

    # ---------- 干预入口 ----------
    def start_intervention(self):
        if self.ramp_in_progress:
            return
        self.ramp_in_progress = True
        rospy.loginfo("[slope] 🚀 开始坡道干预...")

        try:
            self.move_base_client.cancel_all_goals()
            rospy.loginfo("[slope] move_base 目标已取消")

            success = self.execute_ramp_crossing()

            if success:
                if self.clear_costmaps_srv:
                    self.clear_costmaps_srv()
                    rospy.loginfo("[slope] costmap 已清除")

                if self.last_goal is not None:
                    self.last_goal.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(self.last_goal)
                    rospy.loginfo("[slope] ✅ 已重新发布目标 (%.2f, %.2f)，导航已恢复",
                                  self.last_goal.pose.position.x,
                                  self.last_goal.pose.position.y)
                else:
                    rospy.logwarn("[slope] ⚠️ 没有缓存的目标，无法恢复导航")

                self.ramp_handled = True
                rospy.loginfo("[slope] 🎉 坡道任务完成，节点退出")
                rospy.signal_shutdown("坡道完成")
            else:
                rospy.logerr("[slope] ❌ 坡道穿越失败，请人工介入")

        except Exception as e:
            rospy.logerr("[slope] 干预异常: %s", e)
        finally:
            self.ramp_in_progress = False


if __name__ == '__main__':
    try:
        node = SlopeIntervention()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
