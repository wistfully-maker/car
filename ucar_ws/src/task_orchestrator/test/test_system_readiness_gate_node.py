import importlib.util
import json
import sys
import threading
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class Message:
    def __init__(self, data="", stamp=None):
        self.data = data
        self.header = types.SimpleNamespace(stamp=stamp)


class Publisher:
    def __init__(self):
        self.messages = []
        self.error = None
        self.before_publish = None

    def publish(self, message):
        if self.before_publish:
            self.before_publish()
        if self.error:
            raise self.error
        self.messages.append(json.loads(message.data))


def install_node(param_values=None):
    state = types.SimpleNamespace(
        now=10.0, publisher=Publisher(), logs=[], errors=[], params=[], transforms=True,
        action_wait_hook=None, param_errors={}, tf_calls=[],
    )
    rospy = types.ModuleType("rospy")
    defaults = {
        "~readiness_gate/message_max_age": 2.0,
        "~readiness_gate/check_period": 0.5,
        "~readiness_gate/tf_timeout": 0.01,
        "~readiness_gate/action_wait_timeout": 0.01,
        "~readiness_gate/log_interval": 5.0,
        "/move_base/base_global_planner": "global_planner/GlobalPlanner",
        "/move_base/base_local_planner": "teb_local_planner/TebLocalPlannerROS",
    }
    defaults.update(param_values or {})
    def get_param(name, default=None):
        state.params.append(name)
        error = state.param_errors.pop(name, None)
        if error:
            raise error
        return defaults.get(name, default)
    rospy.get_param = get_param
    rospy.get_time = lambda: state.now
    rospy.Duration = lambda value: value
    rospy.Time = lambda value=0: value
    rospy.Publisher = lambda *a, **k: state.publisher
    rospy.Subscriber = lambda *a, **k: types.SimpleNamespace()
    rospy.Timer = lambda *a, **k: types.SimpleNamespace()
    rospy.logwarn = lambda *args: state.logs.append(args)
    rospy.logerr = lambda *args: state.errors.append(args)
    rospy.init_node = lambda *a: None
    rospy.spin = lambda: None

    class Buffer:
        def can_transform(self, *args):
            state.tf_calls.append(args[:2])
            return state.transforms

    tf2 = types.ModuleType("tf2_ros")
    tf2.Buffer = Buffer
    tf2.TransformListener = lambda buffer: types.SimpleNamespace()
    def wait_for_server(timeout):
        if state.action_wait_hook:
            state.action_wait_hook()
        return True
    client = types.SimpleNamespace(wait_for_server=wait_for_server)
    actionlib = types.ModuleType("actionlib")
    actionlib.SimpleActionClient = lambda *a: client
    rosnode = types.ModuleType("rosnode")
    rosnode.rosnode_ping = lambda name, max_count=1, verbose=False: name == "/lidar_loc"

    modules = {
        "rospy": rospy, "tf2_ros": tf2, "actionlib": actionlib, "rosnode": rosnode,
        "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": types.ModuleType("std_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "nav_msgs": types.ModuleType("nav_msgs"), "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
        "move_base_msgs": types.ModuleType("move_base_msgs"), "move_base_msgs.msg": types.ModuleType("move_base_msgs.msg"),
    }
    modules["std_msgs.msg"].String = Message
    modules["sensor_msgs.msg"].LaserScan = type("LaserScan", (), {})
    modules["nav_msgs.msg"].Odometry = type("Odometry", (), {})
    modules["nav_msgs.msg"].OccupancyGrid = type("OccupancyGrid", (), {})
    modules["move_base_msgs.msg"].MoveBaseAction = type("MoveBaseAction", (), {})
    sys.modules.update(modules)
    spec = importlib.util.spec_from_file_location("readiness_node_test", ROOT / "scripts/system_readiness_gate_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SystemReadinessGate(), state


def status(task, state="CHECKING_DEPENDENCIES", version=1):
    return Message(json.dumps({"protocol_version": version, "task_id": task, "state": state, "status": "active", "message": ""}))


class NodeTests(unittest.TestCase):
    @staticmethod
    def _make_fresh(node):
        for callback in (node._on_scan, node._on_odom, node._on_map):
            callback(Message())

    def test_waits_for_valid_checking_status_then_publishes_once_per_task(self):
        node, state = install_node()
        for callback in (node._on_scan, node._on_odom, node._on_map):
            callback(Message(stamp=9.5))
        node._on_timer(None)
        self.assertEqual([], state.publisher.messages)
        node._on_status(Message("bad json"))
        node._on_status(status("t1", version=2))
        node._on_timer(None)
        self.assertEqual([], state.publisher.messages)
        self.assertEqual([], state.tf_calls)
        node._on_status(status("t1"))
        node._on_timer(None)
        node._on_timer(None)
        self.assertEqual([{"protocol_version": 1, "task_id": "t1", "status": "ready"}], state.publisher.messages)
        node._on_status(status("t2"))
        node._on_timer(None)
        self.assertEqual(["t1", "t2"], [m["task_id"] for m in state.publisher.messages])

    def test_leaving_checking_state_stops_polling_and_all_checks_use_private_config(self):
        node, state = install_node()
        node._on_status(status("t1"))
        node._on_status(status("t1", state="ERROR"))
        node._on_timer(None)
        self.assertEqual([], state.publisher.messages)
        for key in ("message_max_age", "check_period", "tf_timeout", "action_wait_timeout", "log_interval"):
            self.assertIn("~readiness_gate/" + key, state.params)

    def test_malformed_identity_and_unknown_state_are_safely_ignored(self):
        node, state = install_node()
        node._on_status(status(" t1 "))
        node._on_timer(None)
        self.assertEqual([], state.publisher.messages)
        node._on_status(status("t1"))
        node._on_status(status("t1", state="NOT_A_REAL_STATE"))
        for callback in (node._on_scan, node._on_odom, node._on_map):
            callback(Message())
        node._on_timer(None)
        self.assertEqual("t1", state.publisher.messages[0]["task_id"])

    def test_leaving_checking_during_health_rpc_is_immediate_and_suppresses_ready(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("t1"))
        entered = threading.Event()
        release = threading.Event()
        state.action_wait_hook = lambda: (entered.set(), release.wait(2.0))
        timer = threading.Thread(target=node._on_timer, args=(None,))
        timer.start()
        self.assertTrue(entered.wait(1.0))
        callback_done = threading.Event()
        callback = threading.Thread(
            target=lambda: (node._on_status(status("t1", state="ERROR")), callback_done.set())
        )
        callback.start()
        self.assertTrue(callback_done.wait(0.2), "status callback blocked behind readiness RPC")
        release.set()
        timer.join(1.0)
        callback.join(1.0)
        self.assertEqual([], state.publisher.messages)

    def test_switching_task_during_check_discards_old_result_and_new_task_can_publish(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("old"))
        entered = threading.Event()
        release = threading.Event()
        state.action_wait_hook = lambda: (entered.set(), release.wait(2.0))
        timer = threading.Thread(target=node._on_timer, args=(None,))
        timer.start()
        self.assertTrue(entered.wait(1.0))
        node._on_status(status("new"))
        release.set()
        timer.join(1.0)
        self.assertEqual([], state.publisher.messages)
        state.action_wait_hook = None
        node._on_timer(None)
        self.assertEqual(["new"], [item["task_id"] for item in state.publisher.messages])

    def test_concurrent_timers_publish_once_for_one_generation(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("t1"))
        entered = threading.Event()
        release = threading.Event()
        state.action_wait_hook = lambda: (entered.set(), release.wait(2.0))
        timers = [threading.Thread(target=node._on_timer, args=(None,)) for _ in range(2)]
        for timer in timers:
            timer.start()
        self.assertTrue(entered.wait(1.0))
        release.set()
        for timer in timers:
            timer.join(1.0)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])

    def test_publish_exception_is_logged_then_retried_once(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("t1"))
        state.publisher.error = RuntimeError("publish failed")
        node._on_timer(None)
        state.publisher.error = None
        node._on_timer(None)
        node._on_timer(None)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])
        self.assertEqual(1, len(state.errors))

    def test_publish_and_state_leave_have_a_linearized_order(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("t1"))
        publish_entered = threading.Event()
        release_publish = threading.Event()
        timeline = []

        def before_publish():
            publish_entered.set()
            release_publish.wait(2.0)
            timeline.append("publish")

        state.publisher.before_publish = before_publish
        timer = threading.Thread(target=node._on_timer, args=(None,))
        timer.start()
        self.assertTrue(publish_entered.wait(1.0))

        def leave():
            node._on_status(status("t1", state="ERROR"))
            timeline.append("leave")

        callback = threading.Thread(target=leave)
        callback.start()
        callback.join(0.2)
        release_publish.set()
        timer.join(1.0)
        callback.join(1.0)
        self.assertEqual(["publish", "leave"], timeline)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])

    def test_latched_map_survives_age_and_clock_rollback(self):
        node, state = install_node()
        self._make_fresh(node)
        state.now = 100.0
        node._on_scan(Message())
        node._on_odom(Message())
        node._on_status(status("t1"))
        node._on_timer(None)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])

        node._on_status(status("t1", state="ERROR"))
        state.now = 1.0
        node._on_status(status("t2"))
        node._on_timer(None)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])
        node._on_scan(Message())
        node._on_odom(Message())
        node._on_timer(None)
        self.assertEqual(["t1", "t2"], [item["task_id"] for item in state.publisher.messages])

    def test_planner_param_exception_is_missing_then_next_timer_retries(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("t1"))
        state.param_errors["/move_base/base_global_planner"] = RuntimeError("master down")
        node._on_timer(None)
        self.assertEqual([], state.publisher.messages)
        self.assertTrue(any("global planner" in str(entry) for entry in state.logs))
        node._on_timer(None)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])

    def test_tf_pairs_are_exact_and_configured(self):
        node, state = install_node({
            "~readiness_gate/map_frame": "world",
            "~readiness_gate/odom_frame": "wheel_odom",
            "~readiness_gate/base_frame": "robot_base",
            "~readiness_gate/laser_frame": "front_laser",
        })
        self._make_fresh(node)
        node._on_status(status("t1"))
        node._on_timer(None)
        self.assertEqual(
            [("world", "wheel_odom"), ("wheel_odom", "robot_base"), ("robot_base", "front_laser")],
            state.tf_calls,
        )

    def test_snapshot_exception_releases_check_and_next_timer_retries(self):
        node, state = install_node()
        self._make_fresh(node)
        node._on_status(status("t1"))
        snapshot = node._snapshot
        node._snapshot = lambda now: (_ for _ in ()).throw(RuntimeError("snapshot boom"))
        node._on_timer(None)
        self.assertEqual([], state.publisher.messages)
        self.assertEqual(0, len(node._checks_in_progress))
        self.assertTrue(any("readiness evaluation error" in str(entry) for entry in state.logs))
        node._snapshot = snapshot
        node._on_timer(None)
        self.assertEqual(["t1"], [item["task_id"] for item in state.publisher.messages])

    def test_published_task_history_is_bounded(self):
        node, state = install_node()
        self._make_fresh(node)
        for index in range(100):
            task_id = "task-%d" % index
            node._on_status(status(task_id))
            node._on_timer(None)
            node._on_status(status(task_id, state="COMPLETE"))
        self.assertLessEqual(len(node._published_task_ids), 64)


if __name__ == "__main__":
    unittest.main()
