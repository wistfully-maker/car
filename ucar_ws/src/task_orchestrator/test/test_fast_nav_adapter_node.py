import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT / "src"
sys.path.insert(0, str(SOURCE_ROOT))


class Obj:
    pass


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


class FakeClient:
    def __init__(self):
        self.goals = []
        self.callbacks = []
        self.cancel_count = 0
        self.send_error = None
        self.cancel_error = None

    def send_goal(self, goal, done_cb):
        if self.send_error:
            raise self.send_error
        self.goals.append(goal)
        self.callbacks.append(done_cb)

    def cancel_goal(self):
        self.cancel_count += 1
        if self.cancel_error:
            raise self.cancel_error


def install_fake_ros():
    state = types.SimpleNamespace(
        now=10.0,
        params={
            "/ucar_fast_nav/pickup_goal": {
                "frame_id": "map",
                "map_sha256": "abc",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.5,
                "position_tolerance": 0.1,
                "yaw_tolerance": 0.2,
            },
            "~fast_nav_adapter/action_timeout": 123.0,
            "~fast_nav_adapter/settle_time": 0.75,
            "~fast_nav_adapter/linear_stop_threshold": 0.04,
            "~fast_nav_adapter/angular_stop_threshold": 0.06,
        },
        publisher=FakePublisher(),
        client=FakeClient(),
        warnings=[],
        errors=[],
        requested_params=[],
    )
    rospy = types.ModuleType("rospy")
    def get_param(name, default=None):
        state.requested_params.append(name)
        return state.params.get(name, default)
    rospy.get_param = get_param
    rospy.get_time = lambda: state.now
    rospy.Publisher = lambda *args, **kwargs: state.publisher
    rospy.Subscriber = lambda *args, **kwargs: Obj()
    rospy.Timer = lambda *args, **kwargs: Obj()
    rospy.Duration = lambda value: value
    rospy.Time = types.SimpleNamespace(now=lambda: state.now)
    rospy.on_shutdown = lambda callback: None
    rospy.logwarn = lambda *args: state.warnings.append(args)
    rospy.logerr = lambda *args: state.errors.append(args)
    rospy.loginfo = lambda *args: None
    rospy.init_node = lambda *args: None
    rospy.spin = lambda: None

    actionlib = types.ModuleType("actionlib")
    actionlib.SimpleActionClient = lambda *args: state.client
    actionlib_msgs = types.ModuleType("actionlib_msgs.msg")
    actionlib_msgs.GoalStatus = types.SimpleNamespace(SUCCEEDED=3)

    class MoveBaseGoal:
        def __init__(self):
            self.target_pose = Obj()
            self.target_pose.header = Obj()
            self.target_pose.pose = Obj()
            self.target_pose.pose.position = Obj()
            self.target_pose.pose.orientation = Obj()

    move_base = types.ModuleType("move_base_msgs.msg")
    move_base.MoveBaseAction = type("MoveBaseAction", (), {})
    move_base.MoveBaseGoal = MoveBaseGoal
    nav = types.ModuleType("nav_msgs.msg")
    nav.Odometry = type("Odometry", (), {})
    std = types.ModuleType("std_msgs.msg")
    std.String = FakeString
    modules = {
        "rospy": rospy,
        "actionlib": actionlib,
        "actionlib_msgs": types.ModuleType("actionlib_msgs"),
        "actionlib_msgs.msg": actionlib_msgs,
        "move_base_msgs": types.ModuleType("move_base_msgs"),
        "move_base_msgs.msg": move_base,
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": nav,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": std,
    }
    sys.modules.update(modules)
    spec = importlib.util.spec_from_file_location(
        "fast_nav_adapter_node_under_test",
        ROOT / "scripts" / "fast_nav_adapter_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, state


def goal(task="t1", goal_id="g1", version=1):
    return FakeString(json.dumps({
        "protocol_version": version, "task_id": task, "goal_id": goal_id
    }))


def cancel(task="t1", reason="operator cancel"):
    return FakeString(json.dumps({
        "protocol_version": 1, "task_id": task, "reason": reason
    }))


def odom(linear=0.0, angular=0.0):
    message = Obj()
    message.twist = Obj()
    message.twist.twist = Obj()
    message.twist.twist.linear = types.SimpleNamespace(x=linear, y=0.0)
    message.twist.twist.angular = types.SimpleNamespace(z=angular)
    return message


class FastNavAdapterNodeTests(unittest.TestCase):
    def setUp(self):
        self.module, self.state = install_fake_ros()
        self.node = self.module.FastNavAdapter()

    def test_reads_waypoint_and_private_adapter_parameters_and_sends_goal(self):
        self.assertEqual(123.0, self.node._action_timeout)
        self.assertEqual(0.04, self.node._detector.linear_threshold)
        self.assertEqual(0.06, self.node._detector.angular_threshold)
        self.assertEqual(0.75, self.node._detector.settle_time)
        self.assertEqual(
            {
                "/ucar_fast_nav/pickup_goal",
                "~fast_nav_adapter/action_timeout",
                "~fast_nav_adapter/settle_time",
                "~fast_nav_adapter/linear_stop_threshold",
                "~fast_nav_adapter/angular_stop_threshold",
            },
            set(self.state.requested_params),
        )
        self.node._on_goal(goal())
        sent = self.state.client.goals[0]
        self.assertEqual("map", sent.target_pose.header.frame_id)
        self.assertEqual(1.0, sent.target_pose.pose.position.x)
        self.assertEqual(2.0, sent.target_pose.pose.position.y)

    def test_duplicate_replace_and_stale_done(self):
        self.node._on_goal(goal())
        old_done = self.state.client.callbacks[0]
        self.node._on_goal(goal())
        self.assertEqual(1, len(self.state.client.goals))
        self.node._on_goal(goal("t2", "g2"))
        self.assertEqual(1, self.state.client.cancel_count)
        self.assertEqual(2, len(self.state.client.goals))
        old_done(4, None)
        self.assertEqual([], self.state.publisher.messages)

    def test_success_waits_for_settle_then_arrives_once(self):
        self.node._on_goal(goal())
        self.state.client.callbacks[0](3, None)
        self.node._on_odom(odom())
        self.state.now += 0.75
        self.node._on_odom(odom())
        self.node._on_odom(odom())
        self.assertEqual(["arrived"], [m["status"] for m in self.state.publisher.messages])

    def test_success_without_odom_and_continuous_motion_time_out_once(self):
        for with_motion in (False, True):
            with self.subTest(with_motion=with_motion):
                module, state = install_fake_ros()
                node = module.FastNavAdapter()
                node._on_goal(goal())
                state.client.callbacks[0](3, None)
                if with_motion:
                    node._on_odom(odom(1.0, 1.0))
                state.now += 301.0
                node._on_timer(None)
                node._on_timer(None)
                self.assertEqual(["failed"], [m["status"] for m in state.publisher.messages])

    def test_cancel_is_strict_and_shutdown_does_not_publish(self):
        self.node._on_goal(goal())
        self.node._on_cancel(cancel(task=" t1 "))
        self.node._on_cancel(cancel(reason=" operator cancel "))
        self.assertEqual(0, self.state.client.cancel_count)
        self.node._on_cancel(cancel())
        self.assertEqual(["failed"], [m["status"] for m in self.state.publisher.messages])
        self.node._on_cancel(cancel())
        self.assertEqual(1, len(self.state.publisher.messages))
        before = list(self.state.publisher.messages)
        self.node._on_shutdown()
        self.assertEqual(before, self.state.publisher.messages)

    def test_action_exceptions_become_one_failure_and_shutdown_only_logs(self):
        self.state.client.send_error = RuntimeError("send boom")
        self.node._on_goal(goal())
        self.assertEqual(["failed"], [m["status"] for m in self.state.publisher.messages])
        self.node._on_timer(None)
        self.assertEqual(1, len(self.state.publisher.messages))

        module, state = install_fake_ros()
        node = module.FastNavAdapter()
        node._on_goal(goal())
        state.client.cancel_error = RuntimeError("cancel boom")
        node._on_shutdown()
        self.assertEqual([], state.publisher.messages)
        self.assertTrue(state.errors)

    def test_replace_cancel_exception_fails_new_goal_without_sending_it(self):
        self.node._on_goal(goal())
        self.state.client.cancel_error = RuntimeError("replace cancel boom")
        self.node._on_goal(goal("t2", "g2"))
        self.assertEqual(1, len(self.state.client.goals))
        self.assertEqual(["failed"], [m["status"] for m in self.state.publisher.messages])
        self.assertEqual("t2", self.state.publisher.messages[0]["task_id"])

    def test_cancel_exception_still_emits_single_cancel_failure(self):
        self.node._on_goal(goal())
        self.state.client.cancel_error = RuntimeError("cancel boom")
        self.node._on_cancel(cancel())
        self.node._on_timer(None)
        self.assertEqual(["failed"], [m["status"] for m in self.state.publisher.messages])

    def test_timeout_wins_over_late_done_and_emits_once(self):
        self.node._on_goal(goal())
        done = self.state.client.callbacks[0]
        self.state.now += 301.0
        self.node._on_timer(None)
        done(3, None)
        done(4, None)
        self.assertEqual(["failed"], [m["status"] for m in self.state.publisher.messages])


if __name__ == "__main__":
    unittest.main()
