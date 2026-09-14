import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class Obj:
    pass


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakePublisher:
    def __init__(self, topic, state):
        self.topic = topic
        self.state = state

    def publish(self, message):
        self.state.published.setdefault(self.topic, []).append(message.data)
        if self.topic == "/stop/motion_mode":
            self.state.events.append(("mode", message.data))


class FakeClient:
    def __init__(self, state):
        self.state = state
        self.goals = []
        self.callbacks = []
        self.cancel_count = 0
        self.send_error = None

    def send_goal(self, goal, done_cb):
        self.state.events.append(("send_goal", goal))
        if self.send_error is not None:
            raise self.send_error
        self.goals.append(goal)
        self.callbacks.append(done_cb)

    def cancel_goal(self):
        self.cancel_count += 1


def install_fake_ros():
    state = types.SimpleNamespace(
        now=10.0,
        published={},
        publishers={},
        publisher_options={},
        events=[],
        warnings=[],
        errors=[],
    )
    state.client = FakeClient(state)

    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda _name, default=None: default
    rospy.get_time = lambda: state.now

    def publisher(topic, *_args, **kwargs):
        instance = FakePublisher(topic, state)
        state.publishers[topic] = instance
        state.publisher_options[topic] = kwargs
        return instance

    rospy.Publisher = publisher
    rospy.Subscriber = lambda *_args, **_kwargs: Obj()
    rospy.Timer = lambda *_args, **_kwargs: Obj()
    rospy.Duration = lambda value: value
    rospy.Time = types.SimpleNamespace(now=lambda: state.now)
    rospy.on_shutdown = lambda _callback: None
    rospy.logwarn = lambda *args: state.warnings.append(args)
    rospy.logerr = lambda *args: state.errors.append(args)
    rospy.loginfo = lambda *_args: None
    rospy.init_node = lambda *_args: None
    rospy.spin = lambda: None

    actionlib = types.ModuleType("actionlib")
    actionlib.SimpleActionClient = lambda *_args: state.client
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
    geometry = types.ModuleType("geometry_msgs.msg")
    geometry.Twist = type("Twist", (), {})
    nav = types.ModuleType("nav_msgs.msg")
    nav.Odometry = type("Odometry", (), {})
    std = types.ModuleType("std_msgs.msg")
    std.String = FakeString
    sys.modules.update({
        "rospy": rospy,
        "actionlib": actionlib,
        "actionlib_msgs": types.ModuleType("actionlib_msgs"),
        "actionlib_msgs.msg": actionlib_msgs,
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": geometry,
        "move_base_msgs": types.ModuleType("move_base_msgs"),
        "move_base_msgs.msg": move_base,
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": nav,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": std,
    })
    spec = importlib.util.spec_from_file_location(
        "line_navigation_adapter_node_under_test",
        ROOT / "scripts" / "line_navigation_adapter_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, state


def goal():
    return FakeString(json.dumps({
        "protocol_version": 1,
        "task_id": "task-1",
        "goal_id": "line-nav-1",
        "pose": {
            "frame_id": "map",
            "x": 0.5,
            "y": -3.1,
            "qz": -0.7,
            "qw": 0.7,
        },
    }))


def cancel():
    return FakeString(json.dumps({
        "protocol_version": 1,
        "task_id": "task-1",
        "reason": "operator_cancel",
    }))


def odom():
    message = Obj()
    message.twist = Obj()
    message.twist.twist = Obj()
    message.twist.twist.linear = types.SimpleNamespace(x=0.0, y=0.0)
    message.twist.twist.angular = types.SimpleNamespace(z=0.0)
    return message


class StopMuxModeLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.module, self.state = install_fake_ros()
        self.node = self.module.LineNavigationAdapter()

    def modes(self):
        return self.state.published.get("/stop/motion_mode", [])

    def test_navigation_mode_precedes_goal_and_settled_success_restores_idle(self):
        self.node._on_goal(goal())
        self.assertIn("/stop/motion_mode", self.state.publishers)
        self.assertFalse(self.state.publisher_options["/stop/motion_mode"]["latch"])
        self.assertEqual("NAVIGATION", self.modes()[-1])
        self.assertEqual(["mode", "send_goal"], [event[0] for event in self.state.events])

        self.state.client.callbacks[0](3, None)
        self.node._on_odom(odom())
        self.state.now += 0.5
        self.node._on_odom(odom())
        self.assertEqual("IDLE", self.modes()[-1])

    def test_all_terminal_paths_restore_idle(self):
        scenarios = ("action_failure", "send_failure", "cancel", "timeout", "shutdown")
        for scenario in scenarios:
            with self.subTest(scenario=scenario):
                module, state = install_fake_ros()
                node = module.LineNavigationAdapter()
                if scenario == "send_failure":
                    state.client.send_error = RuntimeError("send failed")
                node._on_goal(goal())
                if scenario == "action_failure":
                    state.client.callbacks[0](4, None)
                elif scenario == "cancel":
                    node._on_cancel(cancel())
                elif scenario == "timeout":
                    state.now += 301.0
                    node._on_timer(None)
                elif scenario == "shutdown":
                    node._on_shutdown()
                self.assertEqual(
                    ["NAVIGATION", "IDLE"],
                    state.published.get("/stop/motion_mode", []),
                )


if __name__ == "__main__":
    unittest.main()
