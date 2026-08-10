import importlib.util
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def _install_stubs():
    for name in ("rospy", "geometry_msgs", "geometry_msgs.msg", "std_msgs", "std_msgs.msg"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["geometry_msgs.msg"].Twist = type("Twist", (), {})
    sys.modules["std_msgs.msg"].String = type("String", (), {})


def load_module():
    _install_stubs()
    spec = importlib.util.spec_from_file_location(
        "stop_velocity_mux_node_test", ROOT / "scripts" / "stop_velocity_mux_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    module = load_module()
    state = types.SimpleNamespace(
        now=10.0,
        params=[],
        publishers={},
        subscribers={},
        timer_callback=None,
        shutdown=None,
        messages=[],
        errors=[],
        publish_error=None,
    )
    rospy = types.ModuleType("rospy")
    defaults = {
        "~velocity_mux/source_timeout": 0.3,
        "~velocity_mux/check_period": 0.05,
        "~velocity_mux/max_linear_abs": 1.0,
        "~velocity_mux/max_angular_abs": 2.0,
    }
    defaults.update(param_values or {})
    rospy.get_param = lambda name, default=None: (
        state.params.append(name) or defaults.get(name, default)
    )
    rospy.get_time = lambda: state.now
    rospy.Duration = lambda value: value
    rospy.sleep = lambda _value: None

    def publisher(topic, _kind, **_kwargs):
        def publish(message):
            if state.publish_error:
                raise state.publish_error
            state.messages.append((topic, message))
        state.publishers[topic] = publish
        return types.SimpleNamespace(publish=publish)

    rospy.Publisher = publisher
    rospy.Subscriber = lambda topic, _kind, callback, **_kw: (
        state.subscribers.__setitem__(topic, callback)
    )
    rospy.Timer = lambda _duration, callback: setattr(
        state, "timer_callback", types.SimpleNamespace(callback=callback)
    )
    rospy.logerr = lambda *args: state.errors.append(args)
    rospy.logwarn = lambda *args: None
    rospy.loginfo = lambda *args: None
    rospy.on_shutdown = lambda callback: setattr(state, "shutdown", callback)
    rospy.init_node = lambda *_args: None
    rospy.spin = lambda: None

    modules = {
        "rospy": rospy,
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    modules["geometry_msgs.msg"].Twist = Twist
    modules["std_msgs.msg"].String = String
    sys.modules.update(modules)
    spec = importlib.util.spec_from_file_location(
        "stop_velocity_mux_node_wiring_test",
        ROOT / "scripts" / "stop_velocity_mux_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.StopVelocityMuxNode()
    return node, state, module


class VelocityMuxNodeWiringTests(unittest.TestCase):
    def test_publishes_only_stop_topic_and_never_final_cmd_vel(self):
        _node, state, _module = install_node()
        self.assertEqual({"/cmd_vel/stop"}, set(state.publishers))
        self.assertNotIn("/cmd_vel", state.publishers)

    def test_subscribes_only_isolated_sources_and_mode(self):
        _node, state, _module = install_node()
        self.assertEqual(
            {"/cmd_vel/stop_navigation", "/cmd_vel/stop_manual", "/stop/motion_mode"},
            set(state.subscribers),
        )

    def test_mode_switch_publishes_zero_before_motion(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="NAVIGATION"))
        self.assertEqual(1, len(state.messages))
        topic, message = state.messages[-1]
        self.assertEqual("/cmd_vel/stop", topic)
        self.assertEqual(0.0, message.linear.x)
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.5))
        self.assertEqual(0.5, state.messages[-1][1].linear.x)

    def test_manual_source_is_forwarded_in_manual_mode(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="MANUAL"))
        state.subscribers["/cmd_vel/stop_manual"](Twist(x=0.3, z=0.2))
        self.assertEqual(0.3, state.messages[-1][1].linear.x)
        self.assertEqual(0.2, state.messages[-1][1].angular.z)

    def test_unknown_mode_publishes_zero_and_blocks(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="SOMETHING"))
        self.assertEqual(0.0, state.messages[-1][1].linear.x)
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.5))
        self.assertEqual(0.0, state.messages[-1][1].linear.x)

    def test_invalid_values_publish_zero(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="NAVIGATION"))
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=float("nan")))
        self.assertEqual(0.0, state.messages[-1][1].linear.x)

    def test_clock_rollback_publishes_zero(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="NAVIGATION"))
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.5))
        state.now = 5.0
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.4))
        self.assertEqual(0.0, state.messages[-1][1].linear.x)

    def test_publish_exception_fails_closed_without_crash(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="NAVIGATION"))
        state.publish_error = RuntimeError("transport down")
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.5))
        self.assertTrue(state.errors)
        state.publish_error = None
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.5))
        self.assertEqual(0.5, state.messages[-1][1].linear.x)

    def test_shutdown_publishes_zero(self):
        node, state, module = install_node()
        state.shutdown()
        self.assertEqual("/cmd_vel/stop", state.messages[-1][0])
        self.assertEqual(0.0, state.messages[-1][1].linear.x)

    def test_timer_drives_stale_source_zero(self):
        node, state, module = install_node()
        state.subscribers["/stop/motion_mode"](String(data="NAVIGATION"))
        state.subscribers["/cmd_vel/stop_navigation"](Twist(x=0.5))
        state.now = 10.0 + 0.4
        state.timer_callback.callback(None)
        self.assertEqual(0.0, state.messages[-1][1].linear.x)


if __name__ == "__main__":
    unittest.main()
