import importlib.util
import json
import os
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "orchestrator.yaml"
sys.path.insert(0, str(ROOT / "src"))

from task_orchestrator.handoff import NavigationHandoff


def _install_import_stubs():
    """Minimal modules so the node script imports without a ROS install.

    与其他 node 测试一致：无条件替换 sys.modules，保证本文件在
    任何测试顺序下加载脚本都能拿到完整 stub。
    """
    for name in (
        "rospy",
        "actionlib",
        "rosnode",
        "tf2_ros",
        "geometry_msgs",
        "geometry_msgs.msg",
        "std_msgs",
        "std_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
        "move_base_msgs",
        "move_base_msgs.msg",
    ):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["geometry_msgs.msg"].Twist = type("Twist", (), {})
    sys.modules["geometry_msgs.msg"].PoseWithCovarianceStamped = type(
        "PoseWithCovarianceStamped", (), {}
    )
    sys.modules["std_msgs.msg"].String = type("String", (), {})
    sys.modules["nav_msgs.msg"].Odometry = type("Odometry", (), {})
    sys.modules["nav_msgs.msg"].OccupancyGrid = type("OccupancyGrid", (), {})
    sys.modules["sensor_msgs.msg"].LaserScan = type("LaserScan", (), {})
    sys.modules["sensor_msgs.msg"].Image = type("Image", (), {})
    sys.modules["move_base_msgs.msg"].MoveBaseAction = type("MoveBaseAction", (), {})
    actionlib = sys.modules["actionlib"]
    actionlib.GoalStatus = types.SimpleNamespace(
        SUCCEEDED=3, PREEMPTED=2, ABORTED=4, RECALLED=6, LOST=9
    )
    actionlib.SimpleActionClient = lambda *args, **kwargs: types.SimpleNamespace()
    rosnode = sys.modules["rosnode"]
    rosnode.rosnode_ping = lambda *args, **kwargs: False
    rosnode.get_node_names = lambda: []
    tf2 = sys.modules["tf2_ros"]
    tf2.Buffer = type("Buffer", (), {"can_transform": lambda *args: True})
    tf2.TransformListener = lambda *args: None


_install_import_stubs()


def make_config(**overrides):
    config = {
        "cancel_retries": 3,
        "cancel_timeout": 3.0,
        "stop_stable_duration": 0.75,
        "linear_stop_threshold": 0.02,
        "angular_stop_threshold": 0.05,
        "odom_max_age": 0.5,
        "legacy_exit_retries": 5,
        "legacy_exit_timeout": 10.0,
        "readiness_retries": 30,
        "readiness_poll_period": 1.0,
        "total_timeout": 90.0,
    }
    config.update(overrides)
    return config


class FakeClock:
    def __init__(self):
        self.now = [0.0]

    def __call__(self):
        return self.now[0]


class FakeProcess:
    def __init__(self, name):
        self.name = name
        self.terminated = False
        self.alive = True

    def is_alive(self):
        return self.alive

    def terminate(self, _timeout):
        self.terminated = True
        self.alive = False
        return True


class FakeActions:
    """Records lifecycle operations in exact order; publish_zero only counted."""

    def __init__(self):
        self.effects = []
        self.zero_count = 0
        self.diagnostics = []
        self.failures = []
        self.released = []
        self.cancel_ok = True
        self.legacy_stop_ok = True
        self.legacy_absent = True
        self.start_stop_ok = True
        self.readiness_ok = False
        self.legacy_handle = FakeProcess("legacy")
        self.stop_handle = FakeProcess("stop")
        self.modes_published = []
        self.other_handles = [
            FakeProcess(name)
            for name in ("base", "lidar", "camera", "speech", "qr", "llm")
        ]
        self.shutdown_called = False

    def bind(self, _driver):
        pass

    def publish_zero(self):
        self.zero_count += 1

    def wait_stopped(self):
        self.effects.append("wait_stopped")

    def cancel_goals(self):
        self.effects.append("cancel_goals")

    def stop_owned_legacy(self):
        self.effects.append("stop_owned_legacy")
        if self.legacy_stop_ok:
            self.legacy_handle.terminate(1.0)
        return self.legacy_stop_ok

    def verify_legacy_absent(self):
        self.effects.append("verify_legacy_absent")
        return self.legacy_absent

    def start_owned_stop(self):
        self.effects.append("start_owned_stop")
        return self.start_stop_ok

    def wait_stop_ready(self):
        self.effects.append("wait_stop_ready")
        return self.readiness_ok

    def release_task(self, payload, _goal):
        self.effects.append("release_task")
        self.released.append(payload)

    def publish_motion_mode(self, mode):
        self.modes_published.append(mode)

    def publish_diagnostic(self, payload):
        self.diagnostics.append(payload)

    def handoff_failed(self, payload):
        self.effects.append("handoff_failed")
        self.failures.append(payload)

    def shutdown(self):
        self.shutdown_called = True
        for handle in (self.legacy_handle, self.stop_handle):
            if not handle.terminated:
                handle.terminate(1.0)
        self.publish_zero()


def load_module():
    # 其他 node 测试可能已替换 sys.modules 中的模块；exec 前总是
    # 重装完整 stub，保证本文件在任何测试顺序下可加载脚本。
    _install_import_stubs()
    spec = importlib.util.spec_from_file_location(
        "navigation_handoff_supervisor_node_test",
        ROOT / "scripts" / "navigation_handoff_supervisor_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_driver(module, fake, config=None, clock=None):
    machine = NavigationHandoff(make_config(**(config or {})), clock or FakeClock())
    driver = module.NavigationHandoffDriver(machine, fake)
    return driver


GOAL = {
    "protocol_version": 1,
    "task_id": "task-1",
    "goal_id": "delivery-1",
    "target_workshop": "食品加工车间",
    "selected_item": "苹果",
}


def drive_happy_path(module, fake, config=None, clock=None):
    driver = make_driver(module, fake, config, clock)
    fake.readiness_ok = True
    driver.start(dict(GOAL))
    driver.on_cancel_result(True)
    driver.on_odom(0.1, near_zero=True, valid=True)
    driver.on_odom(0.5, near_zero=True, valid=True)
    driver.on_odom(0.9, near_zero=True, valid=True)
    driver.poll()  # legacy absence probe
    driver.poll()  # stop readiness probe
    return driver


class ManagedProcessGroupTests(unittest.TestCase):
    def test_child_roslaunch_inherits_parent_terminal_output(self):
        module = load_module()
        calls = []

        class Proc:
            def poll(self):
                return None

        def fake_popen(command, **kwargs):
            calls.append((command, kwargs))
            return Proc()

        module.subprocess.Popen = fake_popen
        module.ManagedProcessGroup(["roslaunch", "/tmp/stop.launch"])

        self.assertEqual(1, len(calls))
        _command, kwargs = calls[0]
        self.assertNotIn("stdout", kwargs)
        self.assertNotIn("stderr", kwargs)
        self.assertIs(module.subprocess.DEVNULL, kwargs["stdin"])
        self.assertTrue(kwargs["start_new_session"])

    def test_navigation_children_do_not_inherit_llm_secret(self):
        module = load_module()
        calls = []

        class Proc:
            def poll(self):
                return None

        module.subprocess.Popen = lambda command, **kwargs: (
            calls.append((command, kwargs)) or Proc()
        )
        previous = os.environ.get("SPARK_API_PASSWORD")
        os.environ["SPARK_API_PASSWORD"] = "must-not-reach-navigation-child"
        try:
            module.ManagedProcessGroup(["roslaunch", "/tmp/stop.launch"])
            child_env = calls[0][1].get("env", dict(os.environ))
        finally:
            if previous is None:
                os.environ.pop("SPARK_API_PASSWORD", None)
            else:
                os.environ["SPARK_API_PASSWORD"] = previous

        self.assertNotIn("SPARK_API_PASSWORD", child_env)


class HandoffDriverHappyPathTests(unittest.TestCase):
    def test_exact_lifecycle_sequence_and_no_broad_kill(self):
        module = load_module()
        fake = FakeActions()
        drive_happy_path(module, fake)
        self.assertEqual(
            [
                "cancel_goals",
                "wait_stopped",
                "stop_owned_legacy",
                "verify_legacy_absent",
                "start_owned_stop",
                "wait_stop_ready",
                "release_task",
            ],
            fake.effects,
        )
        self.assertNotIn("pkill", fake.effects)
        self.assertNotIn("rosnode kill -a", fake.effects)


class RosHandoffStatusTests(unittest.TestCase):
    def _actions(self, module):
        published = []
        actions = module._RosHandoffActions.__new__(module._RosHandoffActions)
        actions._status_pub = types.SimpleNamespace(
            publish=lambda message: published.append(json.loads(message.data))
        )
        actions.released_messages = []
        actions._release_pub = types.SimpleNamespace(
            publish=lambda message: actions.released_messages.append(
                json.loads(message.data)
            )
        )
        module.String = lambda data: types.SimpleNamespace(data=data)
        return actions, published

    def test_release_reports_correlated_ready(self):
        module = load_module()
        actions, published = self._actions(module)
        actions.release_task(dict(GOAL), dict(GOAL))
        self.assertEqual("ready", published[-1]["status"])
        self.assertEqual("task-1", published[-1]["task_id"])
        self.assertEqual("delivery-1", published[-1]["goal_id"])

    def test_release_payload_satisfies_stop_mission_gate_contract(self):
        module = load_module()
        actions, _published = self._actions(module)
        actions.release_task(dict(GOAL), dict(GOAL))

        self.assertEqual(1, len(actions.released_messages))
        release = actions.released_messages[0]
        self.assertEqual("delivery-1", release["physical_goal_id"])
        self.assertEqual(
            {
                "target_workshop": GOAL["target_workshop"],
                "selected_item": GOAL["selected_item"],
            },
            release["physical"],
        )

    def test_failure_reports_correlated_failed(self):
        module = load_module()
        actions, published = self._actions(module)
        module.rospy.logerr = lambda *args: None
        module.rospy.logwarn = lambda *args: None
        actions.handoff_failed(dict(
            task_id="task-1", goal_id="delivery-1", reason="stop not ready"
        ))
        self.assertEqual(1, len(published))
        self.assertEqual("failed", published[-1]["status"])
        self.assertEqual("stop not ready", published[-1]["message"])

    def test_retry_diagnostics_do_not_use_status_topic(self):
        module = load_module()
        actions, published = self._actions(module)
        module.rospy.logwarn = lambda *args: None
        actions.publish_diagnostic(dict(
            task_id="task-1",
            goal_id="delivery-1",
            stage="readiness",
            reason="retrying",
        ))
        self.assertEqual([], published)

    def test_publish_zero_precedes_every_lifecycle_step(self):
        module = load_module()
        fake = FakeActions()
        drive_happy_path(module, fake)
        self.assertGreaterEqual(fake.zero_count, 6)

    def test_release_carries_correlated_identity(self):
        module = load_module()
        fake = FakeActions()
        drive_happy_path(module, fake)
        self.assertEqual(1, len(fake.released))
        release = fake.released[0]
        self.assertEqual("task-1", release["task_id"])
        self.assertEqual("delivery-1", release["goal_id"])

    def test_release_publishes_stop_navigation_motion_mode(self):
        module = load_module()
        fake = FakeActions()
        drive_happy_path(module, fake)
        self.assertEqual(["STOP_NAVIGATION"], fake.modes_published)

    def test_only_owned_legacy_handle_is_terminated(self):
        module = load_module()
        fake = FakeActions()
        driver = make_driver(module, fake)
        driver.start(dict(GOAL))
        driver.on_cancel_result(True)
        driver.on_odom(0.1, near_zero=True, valid=True)
        driver.on_odom(0.5, near_zero=True, valid=True)
        driver.on_odom(0.9, near_zero=True, valid=True)
        self.assertTrue(fake.legacy_handle.terminated)
        self.assertFalse(fake.stop_handle.terminated)
        for handle in fake.other_handles:
            self.assertFalse(handle.terminated, handle.name)


class RosHandoffPoseTransferTests(unittest.TestCase):
    @staticmethod
    def _message():
        return types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="", stamp=None),
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=0.0
                    ),
                ),
                covariance=[0.0] * 36,
            ),
        )

    def _actions(self, module, transform=None):
        actions = module._RosHandoffActions.__new__(module._RosHandoffActions)
        actions._supervisor = {
            "initial_pose_frame": "map",
            "base_frame": "base_link",
            "pose_snapshot_timeout": 0.2,
            "pose_snapshot_retries": 3,
            "initial_pose_covariance": 0.1,
        }
        actions._captured_initial_pose = None
        actions._initial_pose_pub = types.SimpleNamespace(published=[])
        actions._initial_pose_pub.publish = actions._initial_pose_pub.published.append
        actions._pose_buffer = types.SimpleNamespace(
            lookup_transform=lambda *_args: transform
        )
        module.PoseWithCovarianceStamped = self._message
        module.rospy.Time = lambda value=0: value
        module.rospy.Duration = lambda value: value
        module.rospy.get_rostime = lambda: 123.0
        module.rospy.loginfo = lambda *_args: None
        module.rospy.logwarn = lambda *_args: None
        return actions

    def test_captures_live_map_to_base_pose_and_republishes_same_orientation(self):
        module = load_module()
        transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=-1.31, y=-0.72, z=0.0),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.97, w=-0.24),
            )
        )
        actions = self._actions(module, transform)

        self.assertTrue(actions._capture_initial_pose())
        actions._publish_initial_pose()

        message = actions._initial_pose_pub.published[-1]
        norm = (0.97 ** 2 + (-0.24) ** 2) ** 0.5
        self.assertAlmostEqual(-1.31, message.pose.pose.position.x)
        self.assertAlmostEqual(-0.72, message.pose.pose.position.y)
        self.assertAlmostEqual(0.97 / norm, message.pose.pose.orientation.z)
        self.assertAlmostEqual(-0.24 / norm, message.pose.pose.orientation.w)
        self.assertEqual("map", message.header.frame_id)

    def test_legacy_stack_is_not_stopped_when_live_pose_snapshot_is_missing(self):
        module = load_module()
        actions = self._actions(module)
        actions._pose_buffer.lookup_transform = lambda *_args: (_ for _ in ()).throw(
            RuntimeError("TF unavailable")
        )
        actions._config = {"legacy_exit_timeout": 10.0}
        actions._legacy_group = FakeProcess("legacy")

        self.assertFalse(actions.stop_owned_legacy())
        self.assertTrue(actions._legacy_group.is_alive())

    def test_live_pose_snapshot_retries_transient_tf_misses(self):
        module = load_module()
        transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=-1.31, y=-0.72, z=0.0),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.97, w=-0.24),
            )
        )
        attempts = []

        def lookup(*_args):
            attempts.append(1)
            if len(attempts) < 3:
                raise RuntimeError("transient TF miss")
            return transform

        actions = self._actions(module)
        actions._pose_buffer.lookup_transform = lookup

        self.assertTrue(actions._capture_initial_pose())
        self.assertEqual(3, len(attempts))

class HandoffDriverRetryTests(unittest.TestCase):
    def test_cancel_transient_miss_retries_then_succeeds(self):
        module = load_module()
        fake = FakeActions()
        fake.cancel_ok = False
        driver = make_driver(module, fake)
        driver.start(dict(GOAL))
        driver.on_cancel_result(False)
        self.assertEqual(["cancel_goals", "cancel_goals"], fake.effects)
        self.assertIn("retrying", fake.diagnostics[-1]["reason"])
        fake.cancel_ok = True
        driver.on_cancel_result(False)
        driver.on_cancel_result(True)
        self.assertEqual("wait_stopped", fake.effects[-1])
        self.assertGreaterEqual(fake.zero_count, 3)

    def test_cancel_exhausted_enters_failed_with_zero_and_no_release(self):
        module = load_module()
        fake = FakeActions()
        fake.cancel_ok = False
        driver = make_driver(module, fake, {"cancel_retries": 2})
        driver.start(dict(GOAL))
        driver.on_cancel_result(False)
        driver.on_cancel_result(False)
        driver.on_cancel_result(False)
        self.assertEqual(["handoff_failed"], fake.effects[-1:])
        self.assertEqual("cancel", fake.failures[-1]["stage"])
        self.assertGreaterEqual(fake.zero_count, 3)
        self.assertEqual([], fake.released)

    def test_legacy_still_running_retries_then_succeeds(self):
        module = load_module()
        fake = FakeActions()
        fake.legacy_absent = False
        driver = make_driver(module, fake)
        driver.start(dict(GOAL))
        driver.on_cancel_result(True)
        driver.on_odom(0.1, near_zero=True, valid=True)
        driver.on_odom(0.5, near_zero=True, valid=True)
        driver.on_odom(0.9, near_zero=True, valid=True)
        self.assertEqual("stop_owned_legacy", fake.effects[-1])
        driver.poll()  # 第一次缺席检查失败，进入有界重试
        self.assertEqual("verify_legacy_absent", fake.effects[-1])
        self.assertIn("retrying", fake.diagnostics[-1]["reason"])
        driver.poll()  # 再次失败仍重试，不误放行
        self.assertNotIn("release_task", fake.effects)
        fake.legacy_absent = True
        driver.poll()  # 缺席确认 -> 启动 stop 栈
        self.assertEqual("start_owned_stop", fake.effects[-1])
        fake.readiness_ok = True
        driver.poll()  # 就绪 -> 放行
        self.assertEqual("release_task", fake.effects[-1])

    def test_legacy_exit_exhausted_enters_failed(self):
        module = load_module()
        fake = FakeActions()
        fake.legacy_absent = False
        driver = make_driver(module, fake, {"legacy_exit_retries": 1})
        driver.start(dict(GOAL))
        driver.on_cancel_result(True)
        driver.on_odom(0.1, near_zero=True, valid=True)
        driver.on_odom(0.5, near_zero=True, valid=True)
        driver.on_odom(0.9, near_zero=True, valid=True)
        driver.poll()
        driver.poll()
        self.assertEqual("handoff_failed", fake.effects[-1])
        self.assertEqual("legacy_exit", fake.failures[-1]["stage"])
        self.assertEqual([], fake.released)

    def test_stop_stack_start_failure_enters_failed(self):
        module = load_module()
        fake = FakeActions()
        fake.start_stop_ok = False
        driver = make_driver(module, fake)
        driver.start(dict(GOAL))
        driver.on_cancel_result(True)
        driver.on_odom(0.1, near_zero=True, valid=True)
        driver.on_odom(0.5, near_zero=True, valid=True)
        driver.on_odom(0.9, near_zero=True, valid=True)
        driver.poll()
        self.assertEqual("handoff_failed", fake.effects[-1])
        self.assertEqual("start_stop", fake.failures[-1]["stage"])
        self.assertEqual([], fake.released)

    def test_readiness_never_ready_exhausts_and_fails(self):
        module = load_module()
        fake = FakeActions()
        driver = make_driver(module, fake, {"readiness_retries": 2})
        driver.start(dict(GOAL))
        driver.on_cancel_result(True)
        driver.on_odom(0.1, near_zero=True, valid=True)
        driver.on_odom(0.5, near_zero=True, valid=True)
        driver.on_odom(0.9, near_zero=True, valid=True)
        driver.poll()
        driver.poll()
        driver.poll()
        driver.poll()
        self.assertEqual("handoff_failed", fake.effects[-1])
        self.assertEqual("readiness", fake.failures[-1]["stage"])
        self.assertEqual([], fake.released)
        self.assertGreaterEqual(fake.zero_count, 8)

    def test_moving_or_stale_odom_never_starts_stop(self):
        module = load_module()
        fake = FakeActions()
        driver = make_driver(module, fake)
        driver.start(dict(GOAL))
        driver.on_cancel_result(True)
        driver.on_odom(0.1, near_zero=True, valid=True)
        driver.on_odom(0.3, near_zero=False, valid=True)
        driver.on_odom(0.5, near_zero=True, valid=True)
        driver.on_odom(1.2, near_zero=True, valid=False)
        self.assertNotIn("stop_owned_legacy", fake.effects)
        self.assertEqual("vehicle_moving", fake.diagnostics[-2]["reason"])
        self.assertEqual("odom_stale", fake.diagnostics[-1]["reason"])

    def test_duplicate_goal_does_not_restart(self):
        module = load_module()
        fake = FakeActions()
        driver = make_driver(module, fake)
        driver.start(dict(GOAL))
        effect_count = len(fake.effects)
        driver.start(dict(GOAL))
        driver.start({**GOAL, "goal_id": "delivery-2"})
        self.assertEqual(effect_count, len(fake.effects))

    def test_observations_after_ready_are_ignored(self):
        module = load_module()
        fake = FakeActions()
        driver = drive_happy_path(module, fake)
        count = len(fake.effects)
        driver.on_cancel_result(True)
        driver.poll()
        self.assertEqual(count, len(fake.effects))


class NodeWiringTests(unittest.TestCase):
    @staticmethod
    def _install(param_values=None, actions=None):
        state = types.SimpleNamespace(
            now=10.0,
            params=[],
            publishers={},
            publisher_options={},
            subscribers={},
            timer_callback=None,
            shutdown=None,
            messages=[],
            logs=[],
            errors=[],
        )
        rospy = types.ModuleType("rospy")
        defaults = {
            "~navigation_handoff/cancel_retries": 3,
            "~navigation_handoff/cancel_timeout": 3.0,
            "~navigation_handoff/stop_stable_duration": 0.75,
            "~navigation_handoff/linear_stop_threshold": 0.02,
            "~navigation_handoff/angular_stop_threshold": 0.05,
            "~navigation_handoff/odom_max_age": 0.5,
            "~navigation_handoff/legacy_exit_retries": 5,
            "~navigation_handoff/legacy_exit_timeout": 10.0,
            "~navigation_handoff/readiness_retries": 30,
            "~navigation_handoff/readiness_poll_period": 1.0,
            "~navigation_handoff/total_timeout": 90.0,
            "~navigation_handoff_supervisor/legacy_nav_launch": "legacy.launch",
            "~navigation_handoff_supervisor/stop_integration_launch": "stop.launch",
        }
        defaults.update(param_values or {})
        rospy.get_param = lambda name, default=None: (
            state.params.append(name) or defaults.get(name, default)
        )
        rospy.get_time = lambda: state.now
        rospy.Duration = lambda value: value
        rospy.Time = lambda value=0: value
        rospy.sleep = lambda _value: None

        def publisher(topic, _kind, **_kwargs):
            def publish(message):
                if hasattr(message, "data") and isinstance(message.data, str):
                    state.messages.append((topic, json.loads(message.data)))
                else:
                    state.messages.append((topic, message))
            state.publishers[topic] = publish
            state.publisher_options[topic] = dict(_kwargs)
            return types.SimpleNamespace(publish=publish)

        rospy.Publisher = publisher
        rospy.Subscriber = lambda topic, _kind, callback, **_kw: (
            state.subscribers.__setitem__(topic, callback)
        )
        rospy.Timer = lambda _duration, callback: setattr(
            state, "timer_callback", types.SimpleNamespace(callback=callback)
        )
        rospy.loginfo = lambda *args: state.logs.append(args)
        rospy.logwarn = lambda *args: state.logs.append(args)
        rospy.logerr = lambda *args: state.errors.append(args)
        rospy.on_shutdown = lambda callback: setattr(state, "shutdown", callback)
        rospy.init_node = lambda *_args: None
        rospy.spin = lambda: None

        modules = {
            "rospy": rospy,
            "geometry_msgs": types.ModuleType("geometry_msgs"),
            "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
            "std_msgs": types.ModuleType("std_msgs"),
            "std_msgs.msg": types.ModuleType("std_msgs.msg"),
            "nav_msgs": types.ModuleType("nav_msgs"),
            "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
            "sensor_msgs": types.ModuleType("sensor_msgs"),
            "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
            "move_base_msgs": types.ModuleType("move_base_msgs"),
            "move_base_msgs.msg": types.ModuleType("move_base_msgs.msg"),
            "actionlib": types.ModuleType("actionlib"),
            "rosnode": types.ModuleType("rosnode"),
            "tf2_ros": types.ModuleType("tf2_ros"),
        }
        modules["std_msgs.msg"].String = type("String", (), {})
        modules["geometry_msgs.msg"].Twist = type("Twist", (), {})
        modules["geometry_msgs.msg"].PoseWithCovarianceStamped = type(
            "PoseWithCovarianceStamped", (), {}
        )
        modules["nav_msgs.msg"].Odometry = type("Odometry", (), {})
        modules["nav_msgs.msg"].OccupancyGrid = type("OccupancyGrid", (), {})
        modules["sensor_msgs.msg"].LaserScan = type("LaserScan", (), {})
        modules["sensor_msgs.msg"].Image = type("Image", (), {})
        modules["move_base_msgs.msg"].MoveBaseAction = type("MoveBaseAction", (), {})
        actionlib = modules["actionlib"]
        actionlib.GoalStatus = types.SimpleNamespace(
            SUCCEEDED=3, PREEMPTED=2, ABORTED=4, RECALLED=6, LOST=9
        )
        actionlib.SimpleActionClient = lambda *args, **kwargs: types.SimpleNamespace()
        rosnode = modules["rosnode"]
        rosnode.rosnode_ping = lambda *args, **kwargs: False
        rosnode.get_node_names = lambda: []
        tf2 = modules["tf2_ros"]
        tf2.Buffer = type("Buffer", (), {"can_transform": lambda *args: True})
        tf2.TransformListener = lambda *args: None
        sys.modules.update(modules)
        spec = importlib.util.spec_from_file_location(
            "navigation_handoff_supervisor_node_wiring_test",
            ROOT / "scripts" / "navigation_handoff_supervisor_node.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = module.NavigationHandoffSupervisorNode(actions=actions or FakeActions())
        return node, state, module

    def test_node_declares_handoff_and_release_topics(self):
        _node, state, _module = self._install()
        self.assertIn("/task/navigation_handoff_status", state.publishers)
        self.assertIn("/task/stop_mission_goal", state.publishers)
        self.assertIn("/task/delivery_navigation_goal", state.subscribers)
        self.assertIn("/odom", state.subscribers)

    def test_initial_pose_is_latched_and_configured_for_pickup_area(self):
        _node, state, _module = self._install()
        self.assertTrue(state.publisher_options["/initialpose"]["latch"])
        config = CONFIG.read_text(encoding="utf-8")
        self.assertIn("initial_pose_x: -1.40219", config)
        self.assertIn("initial_pose_y: -0.627908", config)
        self.assertIn("initial_pose_yaw: 0.053792653589793", config)

    def test_live_pose_snapshot_timeout_is_configurable(self):
        _node, state, _module = self._install()
        self.assertIn(
            "~navigation_handoff_supervisor/pose_snapshot_timeout",
            state.params,
        )
        self.assertIn(
            "~navigation_handoff_supervisor/pose_snapshot_retries",
            state.params,
        )

    def test_goal_subscriber_parses_and_starts_handoff(self):
        fake = FakeActions()
        _node, state, _module = self._install(actions=fake)
        state.subscribers["/task/delivery_navigation_goal"](
            types.SimpleNamespace(data=json.dumps(GOAL, ensure_ascii=False))
        )
        self.assertEqual(["cancel_goals"], fake.effects)

    def test_invalid_goal_is_ignored(self):
        fake = FakeActions()
        _node, state, _module = self._install(actions=fake)
        for raw in ("not json", json.dumps({"protocol_version": 2})):
            state.subscribers["/task/delivery_navigation_goal"](
                types.SimpleNamespace(data=raw)
            )
        self.assertEqual([], fake.effects)
        self.assertEqual(
            [],
            [m for t, m in state.messages if t == "/task/stop_mission_goal"],
        )

    def test_shutdown_stops_owned_groups_publishes_zero_no_arrival(self):
        fake = FakeActions()
        _node, state, _module = self._install(actions=fake)
        state.shutdown()
        self.assertTrue(fake.shutdown_called)
        self.assertTrue(fake.legacy_handle.terminated)
        self.assertTrue(fake.stop_handle.terminated)
        for handle in fake.other_handles:
            self.assertFalse(handle.terminated, handle.name)
        self.assertGreaterEqual(fake.zero_count, 1)
        self.assertEqual(
            [],
            [m for t, m in state.messages if t == "/task/stop_mission_goal"],
        )

    def test_odom_subscriber_classifies_speed(self):
        node, state, module = self._install()
        fake = FakeActions()
        node._driver = module.NavigationHandoffDriver(
            NavigationHandoff(make_config(), FakeClock()), fake
        )
        node._driver.start(dict(GOAL))
        node._driver.on_cancel_result(True)
        def odom(stamp, x=0.0, y=0.0, z=0.0):
            return types.SimpleNamespace(
                header=types.SimpleNamespace(stamp=stamp),
                twist=types.SimpleNamespace(
                    twist=types.SimpleNamespace(
                        linear=types.SimpleNamespace(x=x, y=y),
                        angular=types.SimpleNamespace(z=z),
                    )
                ),
            )

        node._on_odom(odom(state.now - 0.1))
        node._on_odom(odom(state.now - 0.1, x=0.05))
        node._on_odom(odom(state.now + 5.0))
        self.assertEqual("vehicle_moving", fake.diagnostics[-2]["reason"])
        self.assertEqual("odom_stale", fake.diagnostics[-1]["reason"])


if __name__ == "__main__":
    unittest.main()
