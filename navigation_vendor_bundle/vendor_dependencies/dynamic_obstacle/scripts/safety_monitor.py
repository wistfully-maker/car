#!/usr/bin/env python3
"""安全监控节点 - 独立于 move_base 运行

订阅 /tracked_objects 和 /odom, 计算 TTC (碰撞时间).
当 TTC < 紧急阈值时直接发布零速绕过 move_base.

发布:
  /safety/cmd_vel (geometry_msgs/Twist)  - 紧急时零速, 安全时透传
  /dynamic_obstacle/ttc (Float32MultiArray) - 调试信息
"""

import math
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from dynamic_obstacle.msg import TrackedObjectArray


# ============== 可调参数 ==============

TTC_EMERGENCY_STOP = 0.5    # 紧急停车阈值 (s)
TTC_SLOWDOWN = 1.0          # 大幅减速阈值 (s)
TTC_CAUTION = 2.0           # 轻微减速阈值 (s)
SAFETY_SLOWDOWN_SPEED = 0.3  # TTC < TTC_SLOWDOWN 时的限速 (m/s)
CAUTION_SPEED = 0.8         # TTC < TTC_CAUTION 时的限速 (m/s)
ROBOT_SAFETY_RADIUS = 0.3   # 机器人安全半径 (m)
OBSTACLE_DEFAULT_RADIUS = 0.3  # 未知障碍物默认半径 (m)


class SafetyMonitor:
    def __init__(self):
        rospy.init_node('safety_monitor')

        # --- 参数 ---
        self.ttc_emergency = rospy.get_param('~ttc_emergency_stop', TTC_EMERGENCY_STOP)
        self.ttc_slowdown = rospy.get_param('~ttc_slowdown', TTC_SLOWDOWN)
        self.ttc_caution = rospy.get_param('~ttc_caution', TTC_CAUTION)
        self.safety_speed = rospy.get_param('~safety_slowdown_speed', SAFETY_SLOWDOWN_SPEED)
        self.caution_speed = rospy.get_param('~caution_speed', CAUTION_SPEED)
        self.robot_radius = rospy.get_param('~robot_safety_radius', ROBOT_SAFETY_RADIUS)
        self.obs_radius = rospy.get_param('~obstacle_default_radius', OBSTACLE_DEFAULT_RADIUS)

        # --- 状态 ---
        self.robot_pose = (0.0, 0.0, 0.0)   # (x, y, theta) in map
        self.robot_vel = (0.0, 0.0)          # (vx, vy) in map
        self.tracked_objects = []
        self.emergency_active = False

        # --- Subscribers ---
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback,
                                          queue_size=1)
        self._tracked_sub = rospy.Subscriber('/tracked_objects', TrackedObjectArray,
                                             self._tracked_callback, queue_size=1)

        # --- Publishers ---
        self._safety_pub = rospy.Publisher('/safety/cmd_vel', Twist, queue_size=1)
        self._ttc_debug_pub = rospy.Publisher('/dynamic_obstacle/ttc',
                                              Float32MultiArray, queue_size=1)

        # --- 定时器 (20Hz) ---
        self._timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)

        # --- 最近一次来自 move_base 的 cmd_vel ---
        self._last_cmd_vel = Twist()
        self._cmd_vel_sub = rospy.Subscriber('/move_base/cmd_vel', Twist,
                                             self._cmd_vel_callback, queue_size=1)

        rospy.loginfo("[safety] 安全监控已就绪")
        rospy.loginfo(f"  紧急停车 TTC < {self.ttc_emergency}s")
        rospy.loginfo(f"  大幅减速 TTC < {self.ttc_slowdown}s → max {self.safety_speed}m/s")
        rospy.loginfo(f"  轻微减速 TTC < {self.ttc_caution}s → max {self.caution_speed}m/s")
        rospy.loginfo(f"  安全半径: {self.robot_radius}m")

    # ----- 回调 -----
    def _odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)  # 简化: 仅使用 z/w
        self.robot_pose = (pos.x, pos.y, yaw)
        self.robot_vel = (msg.twist.twist.linear.x,
                          msg.twist.twist.linear.y)

    def _tracked_callback(self, msg):
        self.tracked_objects = msg.objects

    def _cmd_vel_callback(self, msg):
        self._last_cmd_vel = msg

    # ----- 定时器: 计算 TTC 并决策 -----
    def _timer_callback(self, event):
        if not self.tracked_objects:
            # 无障碍物 → 安全, 不干预
            if self.emergency_active:
                self.emergency_active = False
                rospy.loginfo("[safety] 无障碍物, 解除安全限制")
            return

        min_ttc = float('inf')
        worst_obs = None

        for obj in self.tracked_objects:
            # 跳过未确认的
            if not obj.is_confirmed:
                continue

            ttc = self._compute_ttc(
                self.robot_pose, self.robot_vel,
                (obj.x, obj.y, obj.vx, obj.vy),
                obj.size_x if obj.size_x > 0 else self.obs_radius)

            if ttc < min_ttc:
                min_ttc = ttc
                worst_obs = obj

        # 发布 TTC 调试信息
        self._ttc_debug_pub.publish(Float32MultiArray(
            data=[float(min_ttc), float(len(self.tracked_objects))]))

        # 决策
        if min_ttc < self.ttc_emergency:
            # 紧急停车
            self._emergency_stop(worst_obs, min_ttc)
        elif min_ttc < self.ttc_slowdown:
            # 大幅限速
            self._limit_speed(self.safety_speed)
        elif min_ttc < self.ttc_caution:
            # 轻微限速
            self._limit_speed(self.caution_speed)

    def _emergency_stop(self, obs, ttc):
        if not self.emergency_active:
            self.emergency_active = True
            rospy.logwarn(f"[safety] 紧急停车! TTC={ttc:.2f}s, "
                          f"障碍物 ID={obs.id} at ({obs.x:.2f}, {obs.y:.2f})")
        stop_cmd = Twist()  # 全零
        self._safety_pub.publish(stop_cmd)

    def _limit_speed(self, max_speed):
        self.emergency_active = False
        cmd = Twist()
        cmd.linear.x = min(self._last_cmd_vel.linear.x, max_speed)
        cmd.linear.y = min(self._last_cmd_vel.linear.y, max_speed)
        cmd.angular.z = self._last_cmd_vel.angular.z
        self._safety_pub.publish(cmd)

    # ----- TTC 计算 -----
    def _compute_ttc(self, robot_pose, robot_vel, obs_state, obs_radius):
        """
        robot_pose: (rx, ry, rtheta)
        robot_vel:  (rvx, rvy)
        obs_state:  (ox, oy, ovx, ovy)
        返回最小碰撞时间 (s), 不相撞返回 inf
        """
        rx, ry, _ = robot_pose
        rvx, rvy = robot_vel
        ox, oy, ovx, ovy = obs_state

        # 相对量
        rel_x = ox - rx
        rel_y = oy - ry
        rel_vx = ovx - rvx
        rel_vy = ovy - rvy

        rel_dist = math.hypot(rel_x, rel_y)
        rel_speed = math.hypot(rel_vx, rel_vy)

        combined_radius = self.robot_radius + obs_radius

        # 已经碰撞或非常接近
        if rel_dist <= combined_radius:
            return 0.0

        # 相对静止
        if rel_speed < 0.01:
            return float('inf')

        # 求解二次方程: |rel_pos + t * rel_vel|² = combined_radius²
        a = rel_vx**2 + rel_vy**2
        b = 2.0 * (rel_x * rel_vx + rel_y * rel_vy)
        c = rel_x**2 + rel_y**2 - combined_radius**2

        discriminant = b**2 - 4.0 * a * c
        if discriminant < 0:
            return float('inf')

        sqrt_d = math.sqrt(discriminant)
        t1 = (-b - sqrt_d) / (2.0 * a)
        t2 = (-b + sqrt_d) / (2.0 * a)

        if t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        else:
            return float('inf')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    SafetyMonitor().run()
