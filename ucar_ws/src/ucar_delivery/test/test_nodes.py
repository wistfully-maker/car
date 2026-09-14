"""Phase H: fake-ROS node behavior tests for the ucar_delivery wiring.

Loads the ROS scripts with injected fake rospy/actionlib/message modules and
proves goal handling, phase isolation, motion-mode publication, exclusive
command publishers, the standalone velocity mux and parking command
publication.
"""

import importlib.util
import json
import math
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT.parents[0] / "task_orchestrator" / "src"))

from ucar_delivery.mission import DeliveryMission


class FakeVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class FakePose:
    def __init__(self, position=None, orientation=None):
        self.position = position or FakeVector()
        self.orientation = orientation or FakeVector()


class FakeQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = (
            float(x), float(y), float(z), float(w)
        )


class FakePoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="", stamp=None)
        self.pose = FakePose()


class FakeTwist:
    def __init__(self, x=0.0, z=0.0):
        self.linear = FakeVector(x=x)
        self.angular = FakeVector(z=z)


class FakeGoal:
    def __init__(self):
        self.target_pose = FakePoseStamped()


class FakeOdometry:
    def __init__(self):
        self.pose = types.SimpleNamespace(pose=FakePose())
        self.twist = types.SimpleNamespace(
            twist=FakeTwist()
        )
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: 100.0)
        )


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakeActionClient:
    def __init__(self, action_name, action_type):
        self.action_name = action_name
        self.action_type = action_type
        self.sent_goals = []
        self.cancelled = 0
        self._state = 1

    def send_goal(self, goal):
        self.sent_goals.append(goal)

    def cancel_goal(self):
        self.cancelled += 1

    def get_state(self):
        return self._state

    def wait_for_server(self, _timeout):
        return True


def fake_modules():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.Point = FakeVector
    geometry_msg.Pose = FakePose
    geometry_msg.PoseStamped = FakePoseStamped
    geometry_msg.Quaternion = FakeQuaternion
    geometry_msg.Twist = FakeTwist
    move_base_msgs = types.ModuleType("move_base_msgs")
    move_base_msg = types.ModuleType("move_base_msgs.msg")
    move_base_msg.MoveBaseAction = type("MoveBaseAction", (), {})
    move_base_msg.MoveBaseGoal = FakeGoal
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msg = types.ModuleType("nav_msgs.msg")
    nav_msg.Odometry = FakeOdometry
    std_msgs = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = FakeString
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.Image = type("Image", (), {})
    sensor_msg.LaserScan = type("LaserScan", (), {})
    tf = types.ModuleType("tf.transformations")

    def euler_from_quaternion(quaternion):
        return 0.0, 0.0, 0.0

    def quaternion_from_euler(roll, pitch, yaw):
        return 0.0, 0.0, yaw, 1.0

    tf.euler_from_quaternion = euler_from_quaternion
    tf.quaternion_from_euler = quaternion_from_euler
    return {
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msg,
        "move_base_msgs": move_base_msgs,
        "move_base_msgs.msg": move_base_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msg,
        "tf.transformations": tf,
    }


class RosHarness:
    def __init__(self, param_values=None, action_client=None):
        self.now = [100.0]
        self.messages = {}
        self.publisher_info = {}
        self.subscribers = {}
        self.timers = []
        self.shutdown = None
        self.action_client = action_client or FakeActionClient("move_base", None)
        # 可注入的 laser->map 变换；None 表示 TF 查询失败。
        # tf_history（stamp -> transform）用于验证按 scan 时间戳取变换；
        # 未命中时回退到 tf_transform。
        self.tf_transform = None
        self.tf_history = {}
        self.tf_lookup_calls = []

        rospy = types.ModuleType("rospy")
        rospy.get_time = lambda: self.now[0]
        rospy.get_param = self._make_get_param(param_values or {})
        rospy.get_param_defaults = lambda: None

        def publisher(topic, _kind, **_kwargs):
            self.publisher_info[topic] = _kwargs
            self.messages.setdefault(topic, [])

            def publish(message):
                self.messages[topic].append(message)

            return types.SimpleNamespace(publish=publish)

        rospy.Publisher = publisher

        def subscriber(topic, _kind, callback, **_kwargs):
            self.subscribers[topic] = callback
            return object()

        rospy.Subscriber = subscriber

        def timer(_duration, callback, oneshot=False):
            self.timers.append((callback, oneshot))
            return object()

        rospy.Timer = timer
        rospy.Duration = lambda value: value

        class FakeTime:
            def __new__(cls, stamp=0.0):
                return types.SimpleNamespace(to_sec=lambda: float(stamp))

            @staticmethod
            def now():
                return self.now[0]

        rospy.Time = FakeTime
        rospy.on_shutdown = lambda callback: setattr(self, "shutdown", callback)
        rospy.logwarn = rospy.logerr = rospy.loginfo = rospy.logfatal = lambda *a: None
        rospy.logwarn_throttle = rospy.logerr_throttle = lambda *a: None
        self.rospy = rospy

        modules = {
            "rospy": rospy,
            "actionlib": self._make_actionlib(),
            "tf": self._make_tf(),
        }
        modules.update(fake_modules())
        modules["tf"].transformations = modules["tf.transformations"]
        for name, module in modules.items():
            sys.modules[name] = module
        # allow task_orchestrator imports from the sibling package
        sys.path.insert(0, str(ROOT.parents[0]))

    def _make_actionlib(self):
        actionlib = types.ModuleType("actionlib")
        actionlib.SimpleActionClient = lambda name, kind: self.action_client
        return actionlib

    def _make_tf(self):
        import math as _math
        tf = types.ModuleType("tf")
        harness = self

        class FakeTransformListener:
            # 严格模拟 ROS1 tf.TransformListener.lookupTransform 签名：
            # lookupTransform(target_frame, source_frame, time)，没有
            # timeout= 关键字参数 —— 调用方传入虚构的 timeout= 会抛
            # TypeError（与真实 tf API 一致）。
            def lookupTransform(self, target_frame, source_frame, time):
                harness.tf_lookup_calls.append(
                    (target_frame, source_frame, time)
                )
                stamp = time.to_sec() if hasattr(time, "to_sec") else float(time)
                if harness.tf_history:
                    # 有历史变换表时严格按时间戳取变换：该时刻无变换
                    # 即查询失败（与真实 tf 语义一致）。
                    transform = harness.tf_history.get(stamp)
                    if transform is None:
                        raise Exception("tf: no transform for timestamp")
                else:
                    # 无历史表时回退到单值注入（旧测试）
                    transform = harness.tf_transform
                    if transform is None:
                        raise Exception("tf: unknown frame")
                yaw = transform["yaw"]
                return (
                    (transform["x"], transform["y"], 0.0),
                    (
                        0.0,
                        0.0,
                        _math.sin(yaw / 2.0),
                        _math.cos(yaw / 2.0),
                    ),
                )

        tf.TransformListener = FakeTransformListener
        return tf

    def _make_get_param(self, values):
        def get_param(name, default=None):
            # 模拟 ROS 参数树：支持嵌套路径（如 "~sign_detector/backend"），
            # 也兼容整块读取（如 "~viewpoints"）；每层兼容带/不带 "~" 前缀
            # 的键（既有测试用 "~xxx" 平面键，节点用 "~a/b" 嵌套路径）。
            current = values
            for part in name.lstrip("~").split("/"):
                candidates = (part, "~" + part)
                matched = False
                for key in candidates:
                    if isinstance(current, dict) and key in current:
                        current = current[key]
                        matched = True
                        break
                if not matched:
                    return default
            return current

        return get_param

    def load_script(self, relative, node_class_name):
        spec = importlib.util.spec_from_file_location(
            "ucar_delivery_%s" % node_class_name, ROOT / relative
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = getattr(module, node_class_name)()
        return node, module


def mission_params():
    return {
        "~viewpoints": {
            "physical": [{"x": -1.0, "y": -2.0, "yaw": 0.0}],
            "simulation": [{"x": 1.0, "y": -2.0, "yaw": 3.1}],
        },
        "~timeouts": {
            "navigation": 120.0, "sign_search": 60.0, "sign_align": 30.0,
            "staging_estimate": 30.0, "staging_navigation": 120.0,
            "frame_acquire": 45.0, "frame_align": 30.0, "frame_center": 30.0,
            "approach": 60.0, "final_stop": 30.0, "verify": 15.0,
            "safe_stop": 10.0,
        },
        "~mission": {"viewpoint_count": 1, "viewpoint_max_retries": 2},
        "~nav_supervisor": {
            "action_timeout": 120.0, "cancel_timeout": 10.0, "settle_time": 0.5,
        },
        "~staging_pose": {
            "enabled": True,
            "camera_lidar_yaw_offset_deg": 0.0,
            "bearing_margin_deg": 2.0,
            "min_valid_points": 8,
            "min_inliers": 6,
            "max_fit_residual": 0.04,
            "max_range_spread": 0.50,
            "staging_distance": 0.70,
            "min_staging_travel": 0.15,
            "max_staging_travel": 2.00,
            "estimation_confirmations": 3,
            "estimation_consistency_xy": 0.10,
            "estimation_consistency_yaw_deg": 8.0,
            "staging_estimation_max_retries": 3,
            "staging_navigation_max_retries": 2,
            "tf_timeout": 0.5,
            "max_scan_age": 0.5,
        },
        "~sign_search": {"angular_speed": 0.35},
        "~sign_align": {},
        "~topics": {},
        "~frames": {"map": "map", "laser": "laser_frame"},
        "~move_base_action": "move_base",
    }


class MissionNodeTests(unittest.TestCase):
    def setUp(self):
        self.harness = RosHarness(mission_params())

    def test_delivery_goal_starts_navigation_and_publishes_mode(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        self.assertEqual(1, len(self.harness.action_client.sent_goals))
        sent = self.harness.action_client.sent_goals[0]
        self.assertEqual("map", sent.target_pose.header.frame_id)
        modes = self.harness.messages.get("/ucar_delivery/motion_mode", [])
        self.assertIn("NAVIGATION", [m.data for m in modes])
        self.assertEqual("NAVIGATE_VIEWPOINT", node._mission.state)

    def test_invalid_goal_ignored(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString("not json")
        )
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(json.dumps({
                "protocol_version": 1, "task_id": "task-1", "goal_id": "d",
                "target_workshop": "不存在车间", "selected_item": "苹果",
            }))
        )
        self.assertEqual(0, len(self.harness.action_client.sent_goals))
        self.assertEqual("IDLE", node._mission.state)

    def test_simulation_goal_starts_simulation_phase(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1",
            "goal_id": "simulation-delivery-1",
            "target_workshop": "日用品加工车间", "selected_item": "毛巾",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/simulation_navigation_goal"](
            FakeString(goal)
        )
        self.assertEqual("simulation", node._mission.phase)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))

    def test_other_phase_sign_found_ignored(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        before = node._mission.state
        self.harness.subscribers["/task/delivery_sign_found"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "simulation",
                "task_id": "task-1", "goal_id": "delivery-1",
                "detection": {"confidence": 0.9},
            })
        ))
        self.assertEqual(before, node._mission.state)

    def test_sign_found_stops_search_and_aligns(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        self.harness.action_client._state = 3  # SUCCEEDED
        node._on_tick(None)  # 报告 SUCCEEDED -> supervisor SETTLING
        self.harness.now[0] += 0.6  # settle_time 已过
        node._on_tick(None)  # settle 完成 -> session_result -> SEARCH_SIGN
        self.assertEqual("SEARCH_SIGN", node._mission.state)
        self.harness.subscribers["/task/delivery_sign_found"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "detection": {
                    "confidence": 0.9, "detection_yaw": 0.2,
                    "observed_pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
                },
            })
        ))
        self.assertEqual("ALIGN_SIGN", node._mission.state)
        starts = self.harness.messages.get("/task/delivery_sign_start", [])
        self.assertEqual(1, len(starts))
        self.assertEqual("食品加工车间", json.loads(starts[0].data)["target_workshop"])

    def test_frame_observation_starts_parking(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        node._mission.on_viewpoint_result("physical", True, "")
        node._mission.on_sign_found("physical", {"confidence": 0.9})
        node._mission.on_sign_aligned("physical")
        node._mission.on_staging_pose_estimated(
            "physical", {"x": 1.0, "y": 0.5, "yaw": 0.2}
        )
        node._mission.on_staging_navigation_result("physical", True, "")
        self.assertEqual("ACQUIRE_FRAME", node._mission.state)
        node._on_tick(None)
        self.harness.subscribers["/task/delivery_frame_observation"](
            FakeString(json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "observation": {
                    "timestamp": 100.0, "frame_detected": True,
                    "confidence": 1.0, "near_center_x": 320.0,
                    "far_center_x": 320.0, "visible_boundary_count": 4,
                },
            }))
        )
        self.assertEqual("ALIGN_FRAME", node._mission.state)
        starts = self.harness.messages.get("/task/delivery_parking_start", [])
        self.assertEqual(1, len(starts))


class StagingContractNodeTests(unittest.TestCase):
    """任务 A：节点层锁定动态观察点合同。

    标牌对正后必须停在 ESTIMATE_STAGING_POSE；动态点确认与 staging
    导航成功前，白框搜索与停车不得启动，frame 观测必须被忽略。
    """

    def setUp(self):
        self.harness = RosHarness(mission_params())

    def accept_physical_goal(self, node):
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )

    def to_estimate(self, node):
        self.accept_physical_goal(node)
        node._mission.on_viewpoint_result("physical", True, "")
        node._mission.on_sign_found("physical", {"confidence": 0.9})
        node._mission.on_sign_aligned("physical")
        self.assertEqual(
            "ESTIMATE_STAGING_POSE", node._mission.state
        )

    def test_sign_aligned_does_not_start_frame_search(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.to_estimate(node)
        self.assertEqual(
            0, len(self.harness.messages.get("/task/delivery_frame_start", []))
        )
        self.assertEqual(
            0, len(self.harness.messages.get("/task/delivery_parking_start", []))
        )

    def test_frame_observation_before_staging_confirmed_ignored(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.to_estimate(node)
        before = node._mission.state
        self.harness.subscribers["/task/delivery_frame_observation"](
            FakeString(json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "observation": {
                    "timestamp": 100.0, "frame_detected": True,
                    "confidence": 1.0, "visible_boundary_count": 4,
                },
            }))
        )
        self.assertEqual(before, node._mission.state)
        self.assertEqual(
            0, len(self.harness.messages.get("/task/delivery_parking_start", []))
        )

    def test_frame_search_starts_only_after_staging_navigation_ok(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.to_estimate(node)
        node._mission.on_staging_pose_estimated(
            "physical", {"x": 1.0, "y": 0.5, "yaw": 0.2}
        )
        self.assertEqual(
            "NAVIGATE_STAGING_POSE", node._mission.state
        )
        self.assertEqual(
            0, len(self.harness.messages.get("/task/delivery_frame_start", []))
        )
        node._mission.on_staging_navigation_result("physical", True, "")
        node._flush_outputs()
        self.assertEqual("ACQUIRE_FRAME", node._mission.state)
        starts = self.harness.messages.get("/task/delivery_frame_start", [])
        self.assertEqual(1, len(starts))


def flat_wall_scan(harness, distance=1.0, sector_half_deg=30.0, stamp_offset=0.0):
    """LaserScan fake：正前方 sector 内一面直线墙。"""
    ranges = []
    for deg in range(-90, 91):
        if abs(deg) <= sector_half_deg:
            ranges.append(distance / math.cos(math.radians(deg)))
        else:
            ranges.append(0.0)
    return types.SimpleNamespace(
        ranges=ranges,
        angle_min=-math.pi / 2,
        angle_increment=math.pi / 180.0,
        header=types.SimpleNamespace(
            frame_id="laser_frame",
            stamp=types.SimpleNamespace(
                to_sec=lambda: harness.now[0] - stamp_offset
            )
        ),
    )


def scan_with_stamp(harness, stamp, distance=1.0, sector_half_deg=30.0):
    """LaserScan fake with an explicit header stamp."""
    scan = flat_wall_scan(harness, distance=distance,
                          sector_half_deg=sector_half_deg)
    scan.header.stamp = types.SimpleNamespace(to_sec=lambda: stamp)
    return scan


class StagingIntegrationNodeTests(unittest.TestCase):
    """任务 C：scan + TF -> 连续一致估计 -> staging goal -> 停车交接。

    任何不可信观测（陈旧 scan、TF 失败、检测缺 bearing、估计器拒绝）都
    必须走 mission 的有限重试路径，绝不发送盲目的 move_base 目标。
    """

    def setUp(self):
        self.harness = RosHarness(mission_params())
        self.harness.tf_transform = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def load(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        return node

    def accept_to_estimate(self, node):
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        # 完整走 actionlib 会话：SUCCEEDED + settle -> session_result
        self.harness.action_client._state = 3
        node._on_tick(None)
        self.harness.now[0] += 0.6
        node._on_tick(None)
        self.assertEqual("SEARCH_SIGN", node._mission.state)
        node._mission.on_sign_found("physical", {
            "confidence": 0.9,
            "bearing_rad": 0.0,
            "half_width_rad": 0.5,
        })
        node._mission.on_sign_aligned("physical")
        self.assertEqual("ESTIMATE_STAGING_POSE", node._mission.state)
        node._flush_outputs()

    def feed_scan(self, node, **kwargs):
        self.harness.subscribers["/scan"](flat_wall_scan(self.harness, **kwargs))

    def tick_new_scan(self, node, count):
        """每 tick 前喂一帧新 scan（stamp 随 now 前进）。"""
        for _ in range(count):
            self.harness.now[0] += 0.05
            self.feed_scan(node)
            node._on_tick(None)

    def test_scan_topic_subscribed(self):
        node = self.load()
        self.assertIn("/scan", self.harness.subscribers)

    def test_estimate_confirms_and_sends_staging_goal(self):
        node = self.load()
        self.accept_to_estimate(node)
        self.tick_new_scan(node, 3)  # 3 个不同帧 -> 确认
        self.assertEqual("NAVIGATE_STAGING_POSE", node._mission.state)
        sent = self.harness.action_client.sent_goals
        self.assertEqual(2, len(sent))  # 视点 goal + staging goal
        staging = sent[-1]
        self.assertAlmostEqual(0.3, staging.target_pose.pose.position.x, places=2)
        self.assertAlmostEqual(0.0, staging.target_pose.pose.position.y, places=2)
        self.assertEqual("map", staging.target_pose.header.frame_id)

    def test_staging_goal_sent_once_after_confirmation(self):
        node = self.load()
        self.accept_to_estimate(node)
        self.tick_new_scan(node, 6)
        sent = self.harness.action_client.sent_goals
        self.assertEqual(2, len(sent))  # 只发送一次 staging goal

    def test_stale_scan_waits_without_consuming_retry(self):
        node = self.load()
        self.accept_to_estimate(node)
        self.feed_scan(node, stamp_offset=1.0)  # 超过 max_scan_age=0.5
        node._on_tick(None)
        self.assertEqual("ESTIMATE_STAGING_POSE", node._mission.state)
        self.assertEqual(0, node._mission.staging_estimate_attempt)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))

    def test_tf_failure_blocks_estimate(self):
        node = self.load()
        self.accept_to_estimate(node)
        self.harness.tf_transform = None
        self.feed_scan(node)
        node._on_tick(None)
        self.assertEqual("ALIGN_SIGN", node._mission.state)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))

    def test_missing_detection_bearing_blocks_estimate(self):
        node = self.load()
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        node._mission.on_viewpoint_result("physical", True, "")
        node._mission.on_sign_found("physical", {"confidence": 0.9})
        node._mission.on_sign_aligned("physical")
        self.feed_scan(node)
        node._on_tick(None)
        self.assertEqual("ALIGN_SIGN", node._mission.state)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))

    def test_estimator_rejection_blocks_goal(self):
        node = self.load()
        self.accept_to_estimate(node)
        # 无有效点：扇区外全 0 且墙太近（0.4m < staging_distance 0.7）
        self.feed_scan(node, distance=0.4)
        node._on_tick(None)
        self.assertEqual("ALIGN_SIGN", node._mission.state)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))

    def test_staging_navigation_success_moves_to_frame_search(self):
        node = self.load()
        self.accept_to_estimate(node)
        self.tick_new_scan(node, 3)
        self.assertEqual("NAVIGATE_STAGING_POSE", node._mission.state)
        self.assertEqual(
            0, len(self.harness.messages.get("/task/delivery_frame_start", []))
        )
        self.harness.action_client._state = 1  # 新会话 PENDING
        node._on_tick(None)  # -> ACTIVE
        self.harness.action_client._state = 3  # SUCCEEDED
        node._on_tick(None)  # -> SETTLING
        for _ in range(3):
            odom = FakeOdometry()
            odom.pose.pose.orientation = FakeQuaternion()
            odom.header.stamp = types.SimpleNamespace(
                to_sec=lambda: self.harness.now[0]
            )
            self.harness.subscribers["/odom"](odom)
            self.harness.now[0] += 0.3
        node._on_tick(None)  # session_result -> on_staging_navigation_result
        self.assertEqual("ACQUIRE_FRAME", node._mission.state)
        self.assertEqual(
            1, len(self.harness.messages.get("/task/delivery_frame_start", []))
        )

    def test_staging_navigation_failure_retries_then_fails(self):
        node = self.load()
        self.accept_to_estimate(node)
        self.tick_new_scan(node, 3)
        self.assertEqual("NAVIGATE_STAGING_POSE", node._mission.state)
        self.assertEqual(2, len(self.harness.action_client.sent_goals))

        def abort_goal():
            self.harness.action_client._state = 1  # 新会话 PENDING
            node._on_tick(None)
            self.harness.action_client._state = 4  # ABORTED
            node._on_tick(None)

        abort_goal()  # attempt 1 -> 有界重试（第 3 个 goal）
        self.assertEqual("NAVIGATE_STAGING_POSE", node._mission.state)
        self.assertEqual(3, len(self.harness.action_client.sent_goals))
        abort_goal()  # attempt 2 -> 有界重试（第 4 个 goal）
        abort_goal()  # 重试耗尽 -> 安全失败
        self.assertEqual("FAILED", node._mission.state)
        self.assertEqual(4, len(self.harness.action_client.sent_goals))

    def test_disabled_staging_estimation_fails(self):
        params = mission_params()
        params["~staging_pose"] = dict(params["~staging_pose"], enabled=False)
        self.harness = RosHarness(params)
        node = self.load()
        self.accept_to_estimate(node)
        node._on_tick(None)
        self.assertEqual("ALIGN_SIGN", node._mission.state)


class SignAdapterNodeConstructionTests(unittest.TestCase):
    """回归：真实构造 SignAdapterNode 不得抛异常（此前 __init__ 在
    _fake_backend 赋值前引用导致 AttributeError）。"""

    def test_sign_adapter_node_constructs_without_exception(self):
        self.harness = RosHarness(mission_params())
        node, _module = self.harness.load_script(
            "scripts/sign_adapter_node.py", "SignAdapterNode"
        )
        self.assertIn("/usb_cam/image_raw", self.harness.subscribers)
        self.assertIn("/task/delivery_sign_start", self.harness.subscribers)
        self.assertIn("/task/delivery_sign_fake", self.harness.subscribers)
        self.assertIsNotNone(node._detector)


class StrictTfContractTests(unittest.TestCase):
    """回归：fake TF 严格模拟 ROS1 tf.TransformListener.lookupTransform
    签名（无 timeout= 关键字），并支持按 scan 时间戳取对应变换。"""

    def setUp(self):
        self.harness = RosHarness(mission_params())
        self.harness.tf_transform = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def load(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        return node

    def to_estimate(self, node):
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        node._mission.on_viewpoint_result("physical", True, "")
        node._mission.on_sign_found("physical", {
            "confidence": 0.9,
            "bearing_rad": 0.0,
            "half_width_rad": 0.5,
        })
        node._mission.on_sign_aligned("physical")
        self.assertEqual("ESTIMATE_STAGING_POSE", node._mission.state)
        node._flush_outputs()

    def test_fake_tf_rejects_invented_timeout_keyword(self):
        node = self.load()
        with self.assertRaises(TypeError):
            node._tf_listener.lookupTransform(
                "map", "laser_frame", 0.0, timeout=0.5
            )

    def test_scan_tf_lookup_uses_scan_timestamp(self):
        node = self.load()
        self.harness.tf_history = {
            100.0: {"x": 1.0, "y": 0.0, "yaw": 0.0},
            100.2: {"x": 2.0, "y": 0.0, "yaw": 0.0},
        }
        node._scan_frame = "laser_frame"
        node._scan_stamp = self.harness.rospy.Time(100.2)
        transform = node._lookup_laser_to_map_transform()
        self.assertEqual(2.0, transform["x"])
        node._scan_stamp = self.harness.rospy.Time(100.0)
        transform = node._lookup_laser_to_map_transform()
        self.assertEqual(1.0, transform["x"])

    def test_scan_tf_lookup_uses_header_frame_and_original_stamp(self):
        node = self.load()
        stamp = types.SimpleNamespace(to_sec=lambda: 100.0)
        scan = flat_wall_scan(self.harness)
        scan.header.frame_id = "front_laser_runtime"
        scan.header.stamp = stamp
        self.harness.tf_transform = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        node._on_scan(scan)
        self.assertIsNotNone(node._lookup_laser_to_map_transform())

        target, source, queried_stamp = self.harness.tf_lookup_calls[-1]
        self.assertEqual("map", target)
        self.assertEqual("front_laser_runtime", source)
        self.assertIs(stamp, queried_stamp)

    def test_zero_stamp_or_empty_frame_is_rejected(self):
        node = self.load()
        scan = flat_wall_scan(self.harness)
        scan.header.stamp = types.SimpleNamespace(to_sec=lambda: 0.0)
        node._on_scan(scan)
        self.assertIsNone(node._scan_stamp)

        scan = flat_wall_scan(self.harness)
        scan.header.frame_id = ""
        node._on_scan(scan)
        self.assertIsNone(node._scan_stamp)

    def test_scan_without_timestamp_tf_fails(self):
        node = self.load()
        self.harness.tf_history = {100.0: {"x": 1.0, "y": 0.0, "yaw": 0.0}}
        node._scan_frame = "laser_frame"
        node._scan_stamp = self.harness.rospy.Time(100.5)  # 该时刻无变换
        self.assertIsNone(node._lookup_laser_to_map_transform())

    def test_each_scan_frame_uses_its_own_tf(self):
        node = self.load()
        self.to_estimate(node)
        self.harness.tf_history = {
            100.0: {"x": 1.0, "y": 0.0, "yaw": 0.0},
            100.2: {"x": 2.0, "y": 0.0, "yaw": 0.0},
        }
        # 帧 A（stamp=100.0）必须用 100.0 时刻的变换（x=1.0），
        # 而不是"最新"的 100.2（x=2.0）：观察点 map x = 0.3 + 1.0 = 1.3
        self.harness.subscribers["/scan"](scan_with_stamp(
            self.harness, 100.0
        ))
        node._on_tick(None)
        self.assertEqual(1, node._staging_consistency._streak)
        self.assertAlmostEqual(
            1.3, node._staging_consistency._reference["x"], places=2
        )
        # 帧 B（stamp=100.2）用 100.2 时刻的变换（x=2.0）→ 观察点 x = 2.3；
        # 与帧 A 位姿不一致 → 一致性计数重置为新参考（正确行为）
        self.harness.subscribers["/scan"](scan_with_stamp(
            self.harness, 100.2
        ))
        node._on_tick(None)
        self.assertAlmostEqual(
            2.3, node._staging_consistency._reference["x"], places=2
        )
        self.assertEqual(1, node._staging_consistency._streak)
        # 同一帧重复 tick：不重复累计（streak 保持 1）
        self.harness.subscribers["/scan"](scan_with_stamp(
            self.harness, 100.2
        ))
        node._on_tick(None)
        node._on_tick(None)
        self.assertEqual(1, node._staging_consistency._streak)
        # 无该时间戳 TF 的帧 → 估计失败（安全，不发 goal）
        self.harness.subscribers["/scan"](scan_with_stamp(
            self.harness, 100.4
        ))
        node._on_tick(None)
        self.assertEqual("ALIGN_SIGN", node._mission.state)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))


class ScanDedupTests(unittest.TestCase):
    """回归：同一 scan_stamp 的重复 tick 不得增加一致性计数。"""

    def setUp(self):
        self.harness = RosHarness(mission_params())
        self.harness.tf_transform = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def test_same_scan_stamp_repeated_tick_does_not_accumulate(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        node._flush_outputs()  # 派发 viewpoint goal
        # 完成 viewpoint 导航会话（actionlib SUCCEEDED + settle），
        # 使 supervisor 空闲、staging begin 可用。
        self.harness.action_client._state = 3
        node._on_tick(None)
        self.harness.now[0] += 0.6
        node._on_tick(None)
        self.assertEqual("SEARCH_SIGN", node._mission.state)
        node._mission.on_sign_found("physical", {
            "confidence": 0.9,
            "bearing_rad": 0.0,
            "half_width_rad": 0.5,
        })
        node._mission.on_sign_aligned("physical")
        self.assertEqual("ESTIMATE_STAGING_POSE", node._mission.state)
        node._flush_outputs()
        # 同一帧重复 tick 6 次：一致性计数必须保持 1
        self.harness.subscribers["/scan"](flat_wall_scan(self.harness))
        for _ in range(6):
            node._on_tick(None)
        self.assertEqual(1, node._staging_consistency._streak)
        self.assertEqual(1, len(self.harness.action_client.sent_goals))
        # 新帧到达后才继续累计：第 2 帧后 streak 2
        self.harness.now[0] += 0.05
        self.harness.subscribers["/scan"](flat_wall_scan(self.harness))
        node._on_tick(None)
        self.assertEqual(2, node._staging_consistency._streak)
        # 第 3 个不同帧达到确认阈值：只发一次 staging goal（确认后计数重置）
        self.harness.now[0] += 0.05
        self.harness.subscribers["/scan"](flat_wall_scan(self.harness))
        node._on_tick(None)
        self.assertEqual(
            "NAVIGATE_STAGING_POSE", node._mission.state
        )
        self.assertEqual(2, len(self.harness.action_client.sent_goals))

    def test_failed_scan_cannot_be_reused_after_realign(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1",
            "goal_id": "delivery-1", "target_workshop": "食品加工车间",
            "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        node._mission.on_viewpoint_result("physical", True, "")
        node._mission.on_sign_found("physical", {
            "confidence": 0.9, "bearing_rad": 0.0,
            "half_width_rad": 0.5,
        })
        node._mission.on_sign_aligned("physical")
        node._flush_outputs()

        # This frame has no TF. It may consume exactly one retry.
        self.harness.tf_transform = None
        scan = flat_wall_scan(self.harness)
        self.harness.subscribers["/scan"](scan)
        node._on_tick(None)
        self.assertEqual("ALIGN_SIGN", node._mission.state)
        self.assertEqual(1, node._mission.staging_estimate_attempt)

        # Re-enter estimation without a new scan. The failed frame must not
        # consume another retry or force another alignment cycle.
        node._mission.on_sign_aligned("physical")
        node._flush_outputs()
        node._on_tick(None)
        self.assertEqual("ESTIMATE_STAGING_POSE", node._mission.state)
        self.assertEqual(1, node._mission.staging_estimate_attempt)


class OdometrySettleWiringTests(unittest.TestCase):
    def test_full_odometry_velocity_and_stamp_reach_nav_supervisor(self):
        harness = RosHarness(mission_params())
        node, _module = harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        calls = []
        node._supervisor.on_odometry_velocity = (
            lambda vx, vy, vth, stamp: calls.append((vx, vy, vth, stamp))
        )
        odom = FakeOdometry()
        odom.pose.pose.orientation = FakeQuaternion()
        odom.twist.twist.linear.x = 0.01
        odom.twist.twist.linear.y = -0.02
        odom.twist.twist.angular.z = 0.03
        odom.header.stamp = types.SimpleNamespace(to_sec=lambda: 99.5)

        node._on_odometry(odom)

        self.assertEqual([(0.01, -0.02, 0.03, 99.5)], calls)

class CommandOwnerContractTests(unittest.TestCase):
    """Task 3: exactly one owner per command source topic; no node may
    publish final /cmd_vel or share the other stage's source topic."""

    def setUp(self):
        self.harness = RosHarness(mission_params())

    def test_mission_node_publishes_only_manual_topic(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.assertIn(
            "/cmd_vel/delivery_manual", self.harness.publisher_info
        )
        self.assertNotIn(
            "/cmd_vel/delivery_parking", self.harness.publisher_info
        )
        self.assertNotIn("/cmd_vel", self.harness.publisher_info)

    def test_parking_node_publishes_only_parking_topic(self):
        self.harness = RosHarness({})
        node, _module = self.harness.load_script(
            "scripts/parking_controller_node.py", "ParkingControllerNode"
        )
        self.assertIn(
            "/cmd_vel/delivery_parking", self.harness.publisher_info
        )
        self.assertNotIn(
            "/cmd_vel/delivery_manual", self.harness.publisher_info
        )
        self.assertNotIn("/cmd_vel", self.harness.publisher_info)


class NoPeriodicZeroTests(unittest.TestCase):
    """Task 3: white-frame parking states must not publish periodic manual
    commands; leaving manual control, SAFE_STOP, cancel and shutdown publish
    a one-shot zero."""

    def setUp(self):
        self.harness = RosHarness(mission_params())

    def manual_messages(self):
        return self.harness.messages.get("/cmd_vel/delivery_manual", [])

    def accept_physical_goal(self, node):
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        self.assertEqual(
            "NAVIGATE_VIEWPOINT", node._mission.state
        )

    def drive_to_state(self, node, state):
        self.accept_physical_goal(node)
        mission = node._mission
        mission.on_viewpoint_result("physical", True, "")
        mission.on_sign_found("physical", {"confidence": 0.9})
        mission.on_sign_aligned("physical")
        if state == "ESTIMATE_STAGING_POSE":
            return
        mission.on_staging_pose_estimated(
            "physical", {"x": 1.0, "y": 0.5, "yaw": 0.2}
        )
        if state == "NAVIGATE_STAGING_POSE":
            return
        mission.on_staging_navigation_result("physical", True, "")
        if state == "ACQUIRE_FRAME":
            return
        mission.on_frame_acquired("physical", {"confidence": 0.8})
        if state == "ALIGN_FRAME":
            return
        mission.on_frame_aligned("physical", {"confidence": 0.8})
        if state == "CENTER_FRAME":
            return
        mission.on_frame_centered("physical", {"confidence": 0.8})
        if state == "APPROACH_FRAME":
            return
        mission.on_approach_complete("physical", {"confidence": 0.8})
        if state == "FINAL_STOP":
            return
        mission.on_final_stop_done("physical", {"confidence": 0.8})
        if state == "VERIFY_STOP":
            return
        raise AssertionError("unsupported target state %s" % state)

    def test_parking_states_publish_no_manual_commands(self):
        for state in (
            "ACQUIRE_FRAME", "ALIGN_FRAME", "CENTER_FRAME",
            "APPROACH_FRAME", "FINAL_STOP", "VERIFY_STOP",
        ):
            self.harness = RosHarness(mission_params())
            node, _module = self.harness.load_script(
                "scripts/delivery_mission_node.py", "DeliveryMissionNode"
            )
            self.drive_to_state(node, state)
            baseline = len(self.manual_messages())
            for _ in range(3):
                node._update_manual_commands()
            self.assertEqual(
                baseline, len(self.manual_messages()), state
            )
            self.assertEqual(
                node._mission.state,
                getattr(DeliveryMission, state),
            )

    def test_search_sign_publishes_rotation_command(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.accept_physical_goal(node)
        node._mission.on_viewpoint_result("physical", True, "")
        node._rotating_for_search = True
        baseline = len(self.manual_messages())
        node._update_manual_commands()
        self.assertEqual(baseline + 1, len(self.manual_messages()))
        self.assertEqual(
            0.35, self.manual_messages()[-1].angular.z
        )

    def test_leaving_manual_control_publishes_one_zero(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.accept_physical_goal(node)
        node._mission.on_viewpoint_result("physical", True, "")
        node._mission.on_sign_found("physical", {"confidence": 0.9})
        node._flush_outputs()  # align_sign -> _begin_yaw_align
        node._update_manual_commands()  # ALIGN_SIGN 发布一次
        baseline = len(self.manual_messages())
        self.assertGreater(baseline, 0)
        node._mission.on_sign_aligned("physical")  # 离开手动控制
        node._update_manual_commands()
        self.assertEqual(baseline + 1, len(self.manual_messages()))
        last = self.manual_messages()[-1]
        self.assertEqual(0.0, last.linear.x)
        self.assertEqual(0.0, last.angular.z)

    def test_safe_stop_and_cancel_publish_zero(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.accept_physical_goal(node)
        node._mission.on_viewpoint_result("physical", True, "")
        node._rotating_for_search = True
        node._update_manual_commands()  # 进入手动域
        baseline = len(self.manual_messages())
        node._mission.cancel("operator cancelled")
        node._update_manual_commands()
        self.assertGreater(
            len(self.manual_messages()), baseline
        )
        self.assertEqual(0.0, self.manual_messages()[-1].linear.x)

    def test_shutdown_publishes_zero(self):
        node, _module = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        self.accept_physical_goal(node)
        node._mission.on_viewpoint_result("physical", True, "")
        node._rotating_for_search = True
        node._update_manual_commands()
        baseline = len(self.manual_messages())
        self.harness.shutdown()
        self.assertGreater(
            len(self.manual_messages()), baseline
        )
        self.assertEqual(0.0, self.manual_messages()[-1].linear.x)


class ParkingOwnershipIntegrationTests(unittest.TestCase):
    """Task 6: a valid frame observation moves the parking controller only;
    the mission node publishes nothing on its manual topic during the
    white-frame parking states."""

    def test_frame_observation_commands_come_only_from_parking_controller(self):
        self.harness = RosHarness(mission_params())
        mission_node, _m = self.harness.load_script(
            "scripts/delivery_mission_node.py", "DeliveryMissionNode"
        )
        parking_node, _p = self.harness.load_script(
            "scripts/parking_controller_node.py", "ParkingControllerNode"
        )
        goal = json.dumps({
            "protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
            "target_workshop": "食品加工车间", "selected_item": "苹果",
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_navigation_goal"](
            FakeString(goal)
        )
        mission = mission_node._mission
        mission.on_viewpoint_result("physical", True, "")
        mission.on_sign_found("physical", {"confidence": 0.9})
        mission.on_sign_aligned("physical")
        mission.on_staging_pose_estimated(
            "physical", {"x": 1.0, "y": 0.5, "yaw": 0.2}
        )
        mission.on_staging_navigation_result("physical", True, "")
        mission.on_frame_acquired("physical", {"confidence": 0.8})
        mission_node._flush_outputs()  # dispatch 链，含 parking_start
        self.assertEqual("ALIGN_FRAME", mission.state)
        # 模拟 ROS 转发：parking_start 送达 parking 控制器
        for message in self.harness.messages.get(
            "/task/delivery_parking_start", []
        ):
            self.harness.subscribers["/task/delivery_parking_start"](message)
        self.assertEqual("ALIGN_FRAME", parking_node._controller.state)
        mission_node._update_manual_commands()
        manual_before = len(
            self.harness.messages.get("/cmd_vel/delivery_manual", [])
        )
        observation = json.dumps({
            "protocol_version": 1, "phase": "physical",
            "task_id": "task-1", "goal_id": "delivery-1",
            "observation": {
                "timestamp": self.harness.now[0],
                "frame_detected": True, "confidence": 1.0,
                "near_center_x": 300.0, "far_center_x": 360.0,
                "front_boundary_y": 300.0, "visible_boundary_count": 4,
            },
        }, ensure_ascii=False)
        self.harness.subscribers["/task/delivery_frame_observation"](
            FakeString(observation)
        )
        parking_commands = self.harness.messages.get(
            "/cmd_vel/delivery_parking", []
        )
        self.assertGreater(len(parking_commands), 0)
        self.assertNotEqual(0.0, parking_commands[-1].angular.z)
        # mission 的手动源在此期间没有任何新命令
        mission_node._update_manual_commands()
        self.assertEqual(
            manual_before,
            len(self.harness.messages.get("/cmd_vel/delivery_manual", [])),
        )


class VelocityMuxNodeTests(unittest.TestCase):
    """Task 4: the standalone mux node is the only final /cmd_vel publisher,
    subscribes all isolated sources, applies mode before command and outputs
    zero after source timeout, on shutdown and on invalid input."""

    def setUp(self):
        self.harness = RosHarness({
            "~velocity_mux/source_timeout": 0.3,
            "~velocity_mux/publish_rate": 20.0,
        })

    def load_mux(self):
        return self.harness.load_script(
            "scripts/delivery_velocity_mux_node.py",
            "DeliveryVelocityMuxNode",
        )

    def cmd_vel_messages(self):
        return self.harness.messages.get("/cmd_vel", [])

    def test_wiring_and_single_final_publisher(self):
        node, _module = self.load_mux()
        for topic in (
            "/cmd_vel/delivery_navigation",
            "/cmd_vel/delivery_manual",
            "/cmd_vel/delivery_parking",
            "/ucar_delivery/motion_mode",
        ):
            self.assertIn(topic, self.harness.subscribers)
        self.assertIn("/cmd_vel", self.harness.publisher_info)
        self.assertEqual(1, len(self.cmd_vel_messages()))  # 初始化零
        self.assertEqual(0.0, self.cmd_vel_messages()[0].linear.x)

    def test_mode_change_before_command_returns_zero(self):
        node, _module = self.load_mux()
        self.harness.messages["/cmd_vel"] = []
        self.harness.subscribers["/ucar_delivery/motion_mode"](
            FakeString("PARKING")
        )
        self.assertEqual(1, len(self.cmd_vel_messages()))
        self.assertEqual(0.0, self.cmd_vel_messages()[-1].linear.x)

    def test_parking_command_passes_after_mode(self):
        node, _module = self.load_mux()
        self.harness.messages["/cmd_vel"] = []
        self.harness.subscribers["/ucar_delivery/motion_mode"](
            FakeString("PARKING")
        )
        self.harness.subscribers["/cmd_vel/delivery_parking"](
            FakeTwist(x=0.08, z=-0.2)
        )
        node._on_tick(None)
        last = self.cmd_vel_messages()[-1]
        self.assertEqual(0.08, last.linear.x)
        self.assertEqual(-0.2, last.angular.z)

    def test_wrong_source_command_ignored(self):
        node, _module = self.load_mux()
        self.harness.messages["/cmd_vel"] = []
        self.harness.subscribers["/ucar_delivery/motion_mode"](
            FakeString("PARKING")
        )
        self.harness.subscribers["/cmd_vel/delivery_manual"](
            FakeTwist(x=0.5)
        )
        node._on_tick(None)
        self.assertEqual(0.0, self.cmd_vel_messages()[-1].linear.x)

    def test_source_timeout_publishes_zero(self):
        node, _module = self.load_mux()
        self.harness.messages["/cmd_vel"] = []
        self.harness.subscribers["/ucar_delivery/motion_mode"](
            FakeString("PARKING")
        )
        self.harness.subscribers["/cmd_vel/delivery_parking"](
            FakeTwist(x=0.08)
        )
        node._on_tick(None)
        self.assertEqual(0.08, self.cmd_vel_messages()[-1].linear.x)
        self.harness.now[0] += 0.4  # 超过 source_timeout=0.3
        node._on_tick(None)
        self.assertEqual(0.0, self.cmd_vel_messages()[-1].linear.x)

    def test_invalid_mode_publishes_zero(self):
        node, _module = self.load_mux()
        self.harness.messages["/cmd_vel"] = []
        self.harness.subscribers["/ucar_delivery/motion_mode"](
            FakeString("BOGUS_MODE")
        )
        self.harness.subscribers["/cmd_vel/delivery_navigation"](
            FakeTwist(x=0.3)
        )
        node._on_tick(None)
        self.assertEqual(0.0, self.cmd_vel_messages()[-1].linear.x)

    def test_shutdown_publishes_zero(self):
        node, _module = self.load_mux()
        self.harness.messages["/cmd_vel"] = []
        self.harness.subscribers["/ucar_delivery/motion_mode"](
            FakeString("PARKING")
        )
        self.harness.subscribers["/cmd_vel/delivery_parking"](
            FakeTwist(x=0.08)
        )
        node._on_tick(None)
        self.assertGreater(self.cmd_vel_messages()[-1].linear.x, 0.0)
        self.harness.shutdown()
        self.assertEqual(0.0, self.cmd_vel_messages()[-1].linear.x)


class ParkingControllerNodeTests(unittest.TestCase):
    def setUp(self):
        self.harness = RosHarness({})

    def test_wiring_and_parking_command_publication(self):
        node, _module = self.harness.load_script(
            "scripts/parking_controller_node.py", "ParkingControllerNode"
        )
        for topic in (
            "/task/delivery_parking_start",
            "/task/delivery_frame_observation",
            "/scan",
            "/odom",
        ):
            self.assertIn(topic, self.harness.subscribers)
        self.assertIn("/cmd_vel/delivery_parking", self.harness.publisher_info)

        self.harness.subscribers["/task/delivery_parking_start"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
            })
        ))
        self.harness.subscribers["/task/delivery_frame_observation"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "observation": {
                    "timestamp": 100.0, "frame_detected": True,
                    "confidence": 1.0, "near_center_x": 280.0,
                    "far_center_x": 340.0, "front_boundary_y": 300.0,
                    "visible_boundary_count": 4, "lateral_error": -40.0,
                    "yaw_error": 60.0,
                },
            })
        ))
        commands = self.harness.messages.get("/cmd_vel/delivery_parking", [])
        self.assertTrue(commands)
        last = commands[-1]
        self.assertLess(last.angular.z, 0.0)

    def test_wrong_context_observation_ignored(self):
        node, _module = self.harness.load_script(
            "scripts/parking_controller_node.py", "ParkingControllerNode"
        )
        self.harness.subscribers["/task/delivery_parking_start"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
            })
        ))
        self.harness.subscribers["/task/delivery_frame_observation"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "simulation",
                "task_id": "task-1", "goal_id": "delivery-1",
                "observation": {"frame_detected": True},
            })
        ))
        # 只有启动时的零命令，未产生任何运动命令
        commands = self.harness.messages.get("/cmd_vel/delivery_parking", [])
        self.assertEqual(1, len(commands))
        self.assertEqual(0.0, commands[-1].linear.x)


if __name__ == "__main__":
    unittest.main()
