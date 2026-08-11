import importlib.util
import math
import sys
import threading
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Twist:
    def __init__(self, x=0.0, z=0.0):
        self.linear, self.angular = Vector(x=x), Vector(z=z)


class String:
    def __init__(self, data=""):
        self.data = data


def install_node(param_values=None):
    state = types.SimpleNamespace(now=10.0, messages=[], subscribers={}, shutdown=None,
                                  params=[], publisher_args=None, timer_callback=None)
    rospy = types.ModuleType("rospy")
    rospy.get_time = lambda: state.now
    def get_param(name, default=None):
        state.params.append(name)
        values = {"~velocity_arbiter/source_timeout": 0.3,
                  "~velocity_arbiter/check_period": 0.05}
        values.update(param_values or {})
        return values.get(name, default)
    rospy.get_param = get_param
    def publisher(*args, **kwargs):
        state.publisher_args = (args, kwargs)
        return types.SimpleNamespace(publish=lambda msg: state.messages.append(msg))
    rospy.Publisher = publisher
    def subscribe(topic, _kind, callback, **_kwargs):
        state.subscribers[topic] = callback
        return object()
    rospy.Subscriber = subscribe
    def timer(_duration, callback):
        state.timer_callback = callback
        return object()
    rospy.Timer = timer
    rospy.Duration = lambda value: value
    rospy.on_shutdown = lambda callback: setattr(state, "shutdown", callback)
    rospy.logwarn = rospy.logerr = rospy.loginfo = lambda *a: None
    modules = {"rospy": rospy, "geometry_msgs": types.ModuleType("geometry_msgs"),
               "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
               "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": types.ModuleType("std_msgs.msg")}
    modules["geometry_msgs.msg"].Twist = Twist
    modules["std_msgs.msg"].String = String
    sys.modules.update(modules)
    spec = importlib.util.spec_from_file_location("velocity_arbiter_node_test", ROOT / "scripts/velocity_arbiter_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.VelocityArbiterNode(), state


def load_orchestrator_adapter():
    spec = importlib.util.spec_from_file_location(
        "task_orchestrator_node_cross_layer_test",
        ROOT / "scripts" / "task_orchestrator_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VelocityArbiterNodeTests(unittest.TestCase):
    def test_orchestrator_callback_batches_publish_in_core_order(self):
        _node, _state = install_node()
        adapter = load_orchestrator_adapter()
        outputs, published = [], []
        lock = threading.RLock()
        entered, release = threading.Event(), threading.Event()

        class Publisher:
            def __init__(self, action):
                self.action = action

            def publish(self, message):
                published.append((self.action, message.data))
                if len(published) == 1:
                    entered.set()
                    release.wait(2.0)

        publishers = {
            "publish_motion_mode": Publisher("mode"),
            "publish_status": Publisher("status"),
        }
        def operation(label, mode):
            outputs.extend([("publish_motion_mode", mode),
                            ("publish_status", {"task": label})])

        first = threading.Thread(
            target=lambda: adapter._run_callback(
                lock, outputs, publishers, lambda: operation("A", "NAVIGATION")))
        second = threading.Thread(
            target=lambda: adapter._run_callback(
                lock, outputs, publishers, lambda: operation("B", "IDLE")))
        first.start()
        self.assertTrue(entered.wait(1.0))
        second.start()
        release.set()
        first.join(1.0)
        second.join(1.0)
        self.assertEqual([], outputs)
        self.assertEqual(
            ["NAVIGATION", '{"task": "A"}', "IDLE", '{"task": "B"}'],
            [data for _action, data in published],
        )

    def test_orchestrator_publish_exception_does_not_lock_out_next_callback(self):
        _node, _state = install_node()
        adapter = load_orchestrator_adapter()
        outputs, published = [], []
        lock = threading.RLock()
        failing = types.SimpleNamespace(
            publish=lambda _message: (_ for _ in ()).throw(RuntimeError("boom")))
        good = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        adapter._run_callback(
            lock, outputs, {"publish_motion_mode": failing},
            lambda: outputs.append(("publish_motion_mode", "NAVIGATION")))
        adapter._run_callback(
            lock, outputs, {"publish_motion_mode": good},
            lambda: outputs.append(("publish_motion_mode", "IDLE")))
        self.assertEqual(["IDLE"], published)

    def test_motion_mode_publisher_is_latched(self):
        _node, _state = install_node()
        adapter = load_orchestrator_adapter()
        calls = []
        adapter.rospy.Publisher = lambda *args, **kwargs: (
            calls.append((args, kwargs)) or types.SimpleNamespace())
        adapter._make_publishers()
        motion = next(call for call in calls if call[0][0] == "/task/motion_mode")
        self.assertTrue(motion[1]["latch"])

    def test_bad_configuration_fails_node_startup(self):
        for bad in (True, 0.0, math.inf, "0.3"):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    install_node({"~velocity_arbiter/source_timeout": bad})

    def test_stale_nonzero_cannot_publish_after_mode_zero(self):
        node, state = install_node()
        state.subscribers["/task/motion_mode"](String("NAVIGATION"))
        entered, release = threading.Event(), threading.Event()
        original = node._publish_decision

        def blocked(value, epoch, allow_motion=False, force=False):
            if allow_motion:
                entered.set()
                release.wait(2.0)
            return original(value, epoch, allow_motion, force)

        node._publish_decision = blocked
        source = threading.Thread(
            target=state.subscribers["/cmd_vel/navigation"], args=(Twist(0.5),))
        source.start()
        self.assertTrue(entered.wait(1.0))
        state.subscribers["/task/motion_mode"](String("IDLE"))
        release.set()
        source.join(1.0)
        self.assertEqual(0.0, state.messages[-1].linear.x)
        self.assertFalse(any(message.linear.x == 0.5 for message in state.messages))

    def test_orchestrator_dispatch_modes_drive_real_arbiter_callback(self):
        node, state = install_node()
        self.assertEqual("/cmd_vel", state.publisher_args[0][0])
        self.assertIs(Twist, state.publisher_args[0][1])
        self.assertFalse(state.publisher_args[1]["latch"])
        adapter = load_orchestrator_adapter()

        class Capture:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        capture = Capture()
        for mode, source, speed in (
            ("NAVIGATION", "/cmd_vel/navigation", 1.0),
            ("QR_SEARCH", "/cmd_vel/qr", 0.2),
            ("IDLE", None, None),
        ):
            outputs = [("publish_motion_mode", mode)]
            adapter._dispatch(outputs, {"publish_motion_mode": capture})
            dispatched = capture.messages[-1]
            self.assertEqual(mode, dispatched.data)
            state.subscribers["/task/motion_mode"](dispatched)
            self.assertEqual(mode, node._arbiter.mode)
            if source is not None:
                state.subscribers[source](Twist(speed))
                self.assertEqual(speed, state.messages[-1].linear.x)

        outputs = [("publish_motion_mode", {"unexpected": "object"})]
        adapter._dispatch(outputs, {"publish_motion_mode": capture})
        self.assertEqual("IDLE", capture.messages[-1].data)
        state.subscribers["/task/motion_mode"](capture.messages[-1])
        self.assertEqual("IDLE", node._arbiter.mode)

    def test_stop_navigation_forwards_only_stop_source(self):
        node, state = install_node()
        state.subscribers["/task/motion_mode"](String("STOP_NAVIGATION"))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/navigation"](Twist(8.0))
        self.assertNotEqual(8.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/qr"](Twist(8.0))
        self.assertNotEqual(8.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/stop"](Twist(0.5))
        self.assertEqual(0.5, state.messages[-1].linear.x)
        state.subscribers["/task/motion_mode"](String("NAVIGATION"))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/stop"](Twist(0.5))
        self.assertNotEqual(0.5, state.messages[-1].linear.x)

    def test_orchestrator_dispatches_stop_navigation_mode(self):
        _node, _state = install_node()
        adapter = load_orchestrator_adapter()
        capture = types.SimpleNamespace(messages=[])
        capture.publish = lambda message: capture.messages.append(message)
        adapter._dispatch(
            [("publish_motion_mode", "STOP_NAVIGATION")],
            {"publish_motion_mode": capture},
        )
        self.assertEqual("STOP_NAVIGATION", capture.messages[-1].data)

    def test_line_follow_subscription_is_isolated_from_other_sources(self):
        node, state = install_node()
        self.assertIn("/cmd_vel/line_follow", state.subscribers)
        state.subscribers["/task/motion_mode"](String("LINE_FOLLOW"))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        for source in ("/cmd_vel/navigation", "/cmd_vel/qr", "/cmd_vel/stop"):
            state.subscribers[source](Twist(8.0))
            self.assertNotEqual(8.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/line_follow"](Twist(0.5, 0.2))
        self.assertEqual(0.5, state.messages[-1].linear.x)
        self.assertEqual(0.2, state.messages[-1].angular.z)
        # 模式切换先零速度，随后 line_follow 来源不再放行。
        state.subscribers["/task/motion_mode"](String("NAVIGATION"))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/line_follow"](Twist(0.5))
        self.assertNotEqual(0.5, state.messages[-1].linear.x)

    def test_line_follow_watchdog_invalid_and_shutdown_zero(self):
        node, state = install_node()
        state.subscribers["/task/motion_mode"](String("LINE_FOLLOW"))
        state.subscribers["/cmd_vel/line_follow"](Twist(1.0))
        self.assertEqual(1.0, state.messages[-1].linear.x)
        state.now += 0.31
        state.timer_callback(None)
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/line_follow"](Twist(math.nan))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/line_follow"](Twist(1.0))
        state.shutdown()
        self.assertEqual(0.0, state.messages[-1].linear.x)

    def test_orchestrator_dispatches_line_follow_mode(self):
        _node, _state = install_node()
        adapter = load_orchestrator_adapter()
        capture = types.SimpleNamespace(messages=[])
        capture.publish = lambda message: capture.messages.append(message)
        adapter._dispatch(
            [("publish_motion_mode", "LINE_FOLLOW")],
            {"publish_motion_mode": capture},
        )
        self.assertEqual("LINE_FOLLOW", capture.messages[-1].data)

    def test_start_sources_switch_watchdog_invalid_and_shutdown(self):
        node, state = install_node()
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/task/motion_mode"](String("NAVIGATION"))
        state.subscribers["/cmd_vel/qr"](Twist(8.0))
        self.assertNotEqual(8.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/navigation"](Twist(1.0))
        self.assertEqual(1.0, state.messages[-1].linear.x)
        state.subscribers["/task/motion_mode"](String("QR_SEARCH"))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/qr"](Twist(0.2))
        self.assertEqual(0.2, state.messages[-1].linear.x)
        state.now += 0.31
        state.timer_callback(None)
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.subscribers["/cmd_vel/qr"](Twist(math.nan))
        self.assertEqual(0.0, state.messages[-1].linear.x)
        state.shutdown()
        self.assertEqual(0.0, state.messages[-1].linear.x)
        self.assertIn("~velocity_arbiter/source_timeout", state.params)
        self.assertIn("~velocity_arbiter/check_period", state.params)


if __name__ == "__main__":
    unittest.main()
