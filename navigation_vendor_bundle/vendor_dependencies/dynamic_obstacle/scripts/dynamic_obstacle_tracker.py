#!/usr/bin/env python3
"""动态障碍物检测追踪节点

Pipeline:
  /scan_filtered → 坐标转换(base_footprint) → 静态背景剔除 → 欧氏聚类
  → 卡尔曼追踪 + Hungarian 关联 → 轨迹预测
  → /tracked_objects (自定义消息) + TEB dynamic_obstacles
"""

import math
import time
import numpy as np
from collections import deque

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (Point, Point32, PolygonStamped, Quaternion,
                               TwistWithCovariance, Pose2D)
from std_msgs.msg import Header
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg  # type: ignore
from dynamic_obstacle.msg import TrackedObject, TrackedObjectArray


# ============== 参数（可通过 launch 文件覆盖） ==============

# 聚类
CLUSTER_TOLERANCE = 0.15       # 欧氏聚类半径 (m)
MIN_CLUSTER_SIZE = 3           # 最小聚类点数
MAX_CLUSTER_SIZE = 100         # 最大聚类点数

# 静态/动态判别
STATIC_MAP_THRESHOLD = 0.3     # 距已知障碍物 < 此值 → 可能静态 (m)
VELOCITY_STATIC_THRESHOLD = 0.05  # 速度 < 此值 → 静态 (m/s)

# 追踪器
MIN_DETECTION_CONFIRM = 2      # 确认新障碍物所需连续帧数
MISS_TOLERANCE = 5             # 丢失追踪前最大丢失帧数
MIN_TRACK_LENGTH = 3           # 最短有效追踪长度
MAX_TRACKERS = 20              # 最大活跃追踪器数

# 卡尔曼
PROCESS_NOISE_POS = 0.1        # 位置过程噪声
PROCESS_NOISE_VEL = 0.5        # 速度过程噪声
MEASUREMENT_NOISE = 0.05       # 观测噪声
ASSOCIATION_GATE = 5.991       # 关联门限 (卡方95% 2DOF)

# 预测
PREDICTION_HORIZON = 2.0       # 预测时间范围 (s)
PREDICTION_STEP = 0.1          # 预测步长 (s)


# ============== 简易欧氏聚类 (不依赖 sklearn) ==============

def euclidean_cluster(points, eps, min_samples):
    """简易欧氏聚类, 返回 [{'center': [x,y], 'points': [[x,y],...], 'size': N}, ...]"""
    if len(points) < min_samples:
        return []

    points = np.array(points)
    visited = np.zeros(len(points), dtype=bool)
    clusters = []

    for i in range(len(points)):
        if visited[i]:
            continue
        visited[i] = True
        # 区域查询
        dists = np.hypot(points[:, 0] - points[i, 0],
                         points[:, 1] - points[i, 1])
        neighbors = np.where((dists < eps) & (~visited))[0]

        cluster_inds = [i]
        queue = list(neighbors)
        for idx in queue:
            if visited[idx]:
                continue
            visited[idx] = True
            cluster_inds.append(idx)
            sub_dists = np.hypot(points[:, 0] - points[idx, 0],
                                 points[:, 1] - points[idx, 1])
            sub_neighbors = np.where((sub_dists < eps) & (~visited))[0]
            queue.extend(sub_neighbors)

        if min_samples <= len(cluster_inds) <= MAX_CLUSTER_SIZE:
            cluster_pts = points[cluster_inds]
            center = cluster_pts.mean(axis=0)
            clusters.append({
                'center': center.tolist(),
                'points': cluster_pts.tolist(),
                'size': len(cluster_inds),
            })

    return clusters


# ============== 卡尔曼追踪器 (CV模型) ==============

class KalmanTracker:
    """2D 匀速运动模型卡尔曼追踪器"""

    __slots__ = ('track_id', 'X', 'P', 'F', 'H', 'Q', 'R',
                 'dt', 'miss_count', 'detection_count', 'total_count',
                 'history')

    _next_id = 1

    @classmethod
    def next_id(cls):
        tid = cls._next_id
        cls._next_id += 1
        return tid

    def __init__(self, x, y, dt):
        self.track_id = self.next_id()
        self.dt = dt
        # 状态: [x, y, vx, vy]^T
        self.X = np.array([[x], [y], [0.0], [0.0]])
        self.P = np.diag([0.1, 0.1, 1.0, 1.0])
        # 状态转移
        self._update_F()
        # 观测矩阵
        self.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])
        self._update_Q()
        self.R = np.eye(2) * (MEASUREMENT_NOISE ** 2)

        self.miss_count = 0
        self.detection_count = 1
        self.total_count = 1
        self.history = deque(maxlen=50)

    def _update_F(self):
        dt = self.dt
        self.F = np.array([[1., 0., dt, 0.],
                           [0., 1., 0., dt],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])

    def _update_Q(self):
        dt = self.dt
        q_p, q_v = PROCESS_NOISE_POS, PROCESS_NOISE_VEL
        self.Q = np.array([
            [q_p * dt**4 / 4, 0.,              q_p * dt**3 / 2, 0.],
            [0.,              q_p * dt**4 / 4, 0.,              q_p * dt**3 / 2],
            [q_p * dt**3 / 2, 0.,              q_p * dt**2,     0.],
            [0.,              q_p * dt**3 / 2, 0.,              q_p * dt**2],
        ]) + np.eye(4) * q_v * dt

    def predict(self):
        self.X = self.F @ self.X
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.total_count += 1
        return self.position

    def update(self, z):
        """z: [x, y] 观测"""
        z = np.asarray(z).reshape(2, 1)
        y = z - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.X = self.X + K @ y
        self.P = self.P - K @ self.H @ self.P
        self.miss_count = 0
        self.detection_count += 1
        self.history.append(self.X.copy())

    def mark_missed(self):
        self.miss_count += 1
        self.predict()

    @property
    def position(self):
        return self.X[0, 0], self.X[1, 0]

    @property
    def velocity(self):
        return self.X[2, 0], self.X[3, 0]

    @property
    def is_confirmed(self):
        return self.detection_count >= MIN_DETECTION_CONFIRM

    @property
    def is_old_enough(self):
        return self.total_count >= MIN_TRACK_LENGTH

    @property
    def is_stale(self):
        return self.miss_count > MISS_TOLERANCE

    def mahalanobis(self, z):
        """马氏距离"""
        z = np.asarray(z).reshape(2, 1)
        y = z - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        return float(y.T @ np.linalg.inv(S) @ y)

    def predict_trajectory(self, horizon=PREDICTION_HORIZON, step=PREDICTION_STEP):
        traj = []
        Xp = self.X.copy()
        Fs = np.array([[1., 0., step, 0.],
                       [0., 1., 0., step],
                       [0., 0., 1., 0.],
                       [0., 0., 0., 1.]])
        n_steps = int(horizon / step)
        for _ in range(n_steps):
            Xp = Fs @ Xp
            traj.append((float(Xp[0]), float(Xp[1])))
        return traj


# ============== 主节点 ==============

class DynamicObstacleTracker:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_tracker')

        # --- 参数 ---
        self.cluster_tol = rospy.get_param('~cluster_tolerance', CLUSTER_TOLERANCE)
        self.min_cluster = rospy.get_param('~min_cluster_size', MIN_CLUSTER_SIZE)
        self.static_threshold = rospy.get_param('~static_map_threshold', STATIC_MAP_THRESHOLD)
        self.dt = 0.1  # 默认10Hz, 动态更新

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')

        # --- 静态地图 (缓存) ---
        self.static_map = None       # numpy 2D: occupancy probabilities
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)  # (x, y) of cell (0,0) in world coords
        self.map_width = 0
        self.map_height = 0
        self._map_sub = rospy.Subscriber('/map', OccupancyGrid, self._map_callback,
                                         queue_size=1)

        # --- 追踪器列表 ---
        self.trackers = []           # List[KalmanTracker]

        # --- 时间 ---
        self.last_time = rospy.Time.now()

        # --- Subscribers ---
        self._scan_sub = rospy.Subscriber('/scan_filtered', LaserScan,
                                          self._scan_callback, queue_size=1)

        # --- Publishers ---
        self._tracked_pub = rospy.Publisher('/tracked_objects', TrackedObjectArray,
                                            queue_size=1)
        self._teb_obs_pub = rospy.Publisher(
            '/move_base/TebLocalPlannerROS/dynamic_obstacles',
            ObstacleArrayMsg, queue_size=1)
        self._debug_cluster_pub = rospy.Publisher(
            '/dynamic_obstacle/clusters', PointCloud2, queue_size=1)

        rospy.loginfo("[tracker] 动态障碍物追踪器已就绪")
        rospy.loginfo(f"  聚类半径: {self.cluster_tol}m, 最小点数: {self.min_cluster}")
        rospy.loginfo(f"  静态阈值: {self.static_threshold}m, 最大追踪器: {MAX_TRACKERS}")

    # ----- 静态地图回调 -----
    def _map_callback(self, msg):
        try:
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_resolution = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x,
                               msg.info.origin.position.y)
            data = np.array(msg.data, dtype=np.int8).reshape(
                self.map_height, self.map_width)
            self.static_map = data
            rospy.loginfo(f"[tracker] 静态地图已加载: {self.map_width}x{self.map_height}, "
                          f"分辨率={self.map_resolution:.3f}m")
            # 只加载一次
            self._map_sub.unregister()
        except Exception as e:
            rospy.logerr(f"[tracker] 地图加载失败: {e}")

    # ----- 主回调 -----
    def _scan_callback(self, scan):
        # 更新 dt
        now = scan.header.stamp
        self.dt = max(0.05, min(0.5, (now - self.last_time).to_sec()))
        self.last_time = now

        # Step 1: 将激光点转到 robot_frame
        points_robot = self._transform_scan(scan)
        if points_robot is None or len(points_robot) < self.min_cluster:
            # 没有有效点 → 所有追踪器标为丢失
            for t in self.trackers:
                t.mark_missed()
            self._prune_trackers()
            self._publish_all(now)
            return

        # Step 2: 静态背景剔除
        dynamic_points = self._filter_static(points_robot)

        # Step 3: 欧氏聚类
        clusters = euclidean_cluster(dynamic_points, self.cluster_tol,
                                     self.min_cluster)
        detections = [c['center'] for c in clusters]  # 在 robot_frame 中

        # 发布调试聚类点云
        if self._debug_cluster_pub.get_num_connections() > 0:
            self._publish_clusters_debug(clusters, now)

        # Step 4: 卡尔曼预测
        for t in self.trackers:
            t._update_F()      # 用当前 dt 更新状态转移矩阵
            t._update_Q()
            t.predict()

        # Step 5: Hungarian 数据关联
        matches, unmatched_det, unmatched_trk = self._associate(detections)

        # Step 6: 更新匹配的追踪器
        for trk_idx, det_idx in matches:
            self.trackers[trk_idx].update(detections[det_idx])

        # Step 7: 标记未匹配的追踪器
        for trk_idx in unmatched_trk:
            self.trackers[trk_idx].mark_missed()

        # Step 8: 为未匹配的检测创建新追踪器
        # 先转为 map 坐标再初始化
        for det_idx in unmatched_det:
            if len(self.trackers) >= MAX_TRACKERS:
                break
            det_robot = detections[det_idx]
            det_map = self._robot_to_map(det_robot, now)
            if det_map is not None:
                self.trackers.append(KalmanTracker(det_map[0], det_map[1], self.dt))

        # Step 9: 清理失效追踪器
        self._prune_trackers()

        # Step 10: 发布
        self._publish_all(now)

    # ----- 坐标转换 -----
    def _transform_scan(self, scan):
        """将 LaserScan 转为 robot_frame 下的点列表 [(x,y), ...]"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame, scan.header.frame_id, scan.header.stamp,
                rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"[tracker] TF lookup 失败: {e}")
            return None

        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                # 极坐标 → 直角坐标
                x_laser = r * math.cos(angle)
                y_laser = r * math.sin(angle)
                # TF 变换
                x_robot = x_laser + transform.transform.translation.x
                y_robot = y_laser + transform.transform.translation.y
                # 简化: 忽略旋转 (激光雷达通常与机器人朝向一致)
                yaw = 2.0 * math.atan2(transform.transform.rotation.z,
                                       transform.transform.rotation.w)
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                x_rot = x_robot * cos_yaw - y_robot * sin_yaw
                y_rot = x_robot * sin_yaw + y_robot * cos_yaw
                points.append((x_rot, y_rot))
            angle += scan.angle_increment
        return points

    def _robot_to_map(self, pt_robot, stamp):
        """将 robot_frame 下的点转换到 map_frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, stamp,
                rospy.Duration(0.2))
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            yaw = 2.0 * math.atan2(transform.transform.rotation.z,
                                   transform.transform.rotation.w)
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            x_map = tx + pt_robot[0] * cos_yaw - pt_robot[1] * sin_yaw
            y_map = ty + pt_robot[0] * sin_yaw + pt_robot[1] * cos_yaw
            return (x_map, y_map)
        except Exception:
            return None

    # ----- 静态背景剔除 -----
    def _filter_static(self, points):
        """剔除靠近已知地图障碍物的点, 返回动态候选点"""
        if self.static_map is None:
            # 没有地图 → 所有点都视为动态候选
            return points

        dynamic = []
        ox, oy = self.map_origin
        res = self.map_resolution
        search_radius = int(self.static_threshold / res) + 1
        th2 = self.static_threshold ** 2

        for x, y in points:
            map_x = int((x - ox) / res)
            map_y = int((y - oy) / res)
            is_static = False

            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    mx, my = map_x + dx, map_y + dy
                    if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                        if self.static_map[my, mx] >= 100:  # occupied
                            dist2 = (dx * res) ** 2 + (dy * res) ** 2
                            if dist2 < th2:
                                is_static = True
                                break
                if is_static:
                    break
            if not is_static:
                dynamic.append((x, y))
        return dynamic

    # ----- 数据关联 (Hungarian) -----
    def _associate(self, detections):
        """简易贪心关联 (等价于 Hungarian 对于小规模场景)"""
        if not detections or not self.trackers:
            return [], list(range(len(detections))), list(range(len(self.trackers)))

        # 构建代价矩阵
        n_trk, n_det = len(self.trackers), len(detections)
        cost = np.full((n_trk, n_det), np.inf)
        for i, t in enumerate(self.trackers):
            for j, d in enumerate(detections):
                md = t.mahalanobis(d)
                if md < ASSOCIATION_GATE:
                    cost[i, j] = md

        # 贪心匹配 (最小代价优先)
        matches = []
        used_det = set()
        used_trk = set()

        # 展平并按代价排序
        candidates = []
        for i in range(n_trk):
            for j in range(n_det):
                if cost[i, j] < np.inf:
                    candidates.append((cost[i, j], i, j))
        candidates.sort()

        for _, ti, di in candidates:
            if ti not in used_trk and di not in used_det:
                matches.append((ti, di))
                used_trk.add(ti)
                used_det.add(di)

        unmatched_det = [j for j in range(n_det) if j not in used_det]
        unmatched_trk = [i for i in range(n_trk) if i not in used_trk]
        return matches, unmatched_det, unmatched_trk

    # ----- 清理 -----
    def _prune_trackers(self):
        self.trackers = [t for t in self.trackers
                         if not t.is_stale and t.miss_count <= MISS_TOLERANCE]
        # 如果超出上限, 移除最不确认的
        if len(self.trackers) > MAX_TRACKERS:
            self.trackers.sort(key=lambda t: t.is_confirmed)
            self.trackers = self.trackers[-MAX_TRACKERS:]

    # ----- 发布 -----
    def _publish_all(self, stamp):
        active = [t for t in self.trackers if t.is_old_enough]
        if not active:
            # 发布空消息
            self._tracked_pub.publish(TrackedObjectArray(
                header=Header(stamp=stamp, frame_id=self.map_frame)))
            self._teb_obs_pub.publish(ObstacleArrayMsg(
                header=Header(stamp=stamp, frame_id=self.map_frame)))
            return

        # 自定义消息
        tracked_array = TrackedObjectArray(
            header=Header(stamp=stamp, frame_id=self.map_frame))

        # TEB 动态障碍物
        teb_array = ObstacleArrayMsg(
            header=Header(stamp=stamp, frame_id=self.map_frame))

        for t in active:
            x, y = t.position
            vx, vy = t.velocity
            heading = math.atan2(vy, vx) if abs(vx) > 0.01 or abs(vy) > 0.01 else 0.0
            pred = t.predict_trajectory()

            # TrackedObject
            obj = TrackedObject()
            obj.id = t.track_id
            obj.x = x
            obj.y = y
            obj.vx = vx
            obj.vy = vy
            obj.heading = heading
            obj.size_x = 0.3
            obj.size_y = 0.3
            obj.is_confirmed = t.is_confirmed
            obj.track_length = t.total_count
            obj.predicted_path = [Point(x=px, y=py, z=0) for px, py in pred]
            tracked_array.objects.append(obj)

            # TEB ObstacleMsg
            teb_obj = ObstacleMsg()
            teb_obj.id = t.track_id
            # 当前中心点
            teb_obj.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)]
            # 朝向
            q = quaternion_from_euler(0, 0, heading)
            teb_obj.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            # 速度
            teb_obj.velocities.twist.linear.x = vx
            teb_obj.velocities.twist.linear.y = vy
            teb_obj.velocities.twist.linear.z = 0.0
            teb_array.obstacles.append(teb_obj)

        self._tracked_pub.publish(tracked_array)
        self._teb_obs_pub.publish(teb_array)

    def _publish_clusters_debug(self, clusters, stamp):
        """发布聚类结果为 PointCloud2 用于 RViz 调试"""
        pts = []
        for c in clusters:
            pts.extend(c['points'])
        if not pts:
            return
        cloud = create_cloud_xyz32(
            Header(stamp=stamp, frame_id=self.robot_frame),
            [(p[0], p[1], 0.0) for p in pts])
        self._debug_cluster_pub.publish(cloud)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    DynamicObstacleTracker().run()
