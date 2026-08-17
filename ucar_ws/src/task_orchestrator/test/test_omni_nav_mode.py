import importlib.util
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from task_orchestrator.handoff import navigation_mode_launch_arg


def _install_import_stubs():
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
        if name not in sys.modules:
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
    sys.modules["move_base_msgs.msg"].MoveBaseAction = type(
        "MoveBaseAction", (), {}
    )
    actionlib = sys.modules["actionlib"]
    actionlib.GoalStatus = types.SimpleNamespace(
        SUCCEEDED=3, PREEMPTED=2, ABORTED=4, RECALLED=6, LOST=9
    )
    actionlib.SimpleActionClient = lambda *args, **kwargs: types.SimpleNamespace()
    sys.modules["rosnode"].rosnode_ping = lambda *args, **kwargs: False
    sys.modules["rosnode"].get_node_names = lambda: []
    sys.modules["tf2_ros"].Buffer = type(
        "Buffer", (), {"can_transform": lambda *args: True}
    )
    sys.modules["tf2_ros"].TransformListener = lambda *args: None


class NavigationModeLaunchArgTests(unittest.TestCase):
    def test_unset_modes_return_no_arguments(self):
        self.assertEqual([], navigation_mode_launch_arg(None))
        self.assertEqual([], navigation_mode_launch_arg(""))

    def test_valid_modes_become_single_launch_argument(self):
        self.assertEqual(
            ["navigation_mode:=differential"],
            navigation_mode_launch_arg("differential"),
        )
        self.assertEqual(
            ["navigation_mode:=omni"], navigation_mode_launch_arg("omni")
        )

    def test_rejects_whitespace_and_non_text(self):
        for bad in (" omni", "omni ", "om ni", "\tomni", 123, True):
            with self.assertRaises(ValueError):
                navigation_mode_launch_arg(bad)


class TaskOrchestratorLaunchModeWiringTests(unittest.TestCase):
    def test_launch_declares_empty_mode_args_by_default(self):
        root = ET.parse(
            ROOT / "launch" / "task_orchestrator.launch"
        ).getroot()
        args = {
            a.attrib["name"]: a.attrib.get("default")
            for a in root.findall("arg")
        }
        self.assertEqual("", args["legacy_nav_mode"])
        self.assertEqual("", args["stop_integration_nav_mode"])

    def test_supervisor_node_receives_mode_params(self):
        root = ET.parse(
            ROOT / "launch" / "task_orchestrator.launch"
        ).getroot()
        node = next(
            n for n in root.findall("node")
            if n.attrib["name"] == "navigation_handoff_supervisor"
        )
        params = {
            p.attrib["name"]: p.attrib["value"]
            for p in node.findall("param")
        }
        self.assertEqual(
            "$(arg legacy_nav_mode)",
            params["navigation_handoff_supervisor/legacy_nav_mode"],
        )
        self.assertEqual(
            "$(arg stop_integration_nav_mode)",
            params["navigation_handoff_supervisor/stop_integration_nav_mode"],
        )


class _FakeProcess:
    def poll(self):
        return None


class RunnerStartArgsTests(unittest.TestCase):
    def _load_node_module(self):
        _install_import_stubs()
        spec = importlib.util.spec_from_file_location(
            "navigation_handoff_supervisor_node",
            ROOT / "scripts" / "navigation_handoff_supervisor_node.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _runner_with_calls(self, module):
        calls = []
        module.subprocess.Popen = (
            lambda command, **kwargs: calls.append(command) or _FakeProcess()
        )
        return module.RosLaunchProcessRunner(), calls

    def test_runner_appends_navigation_mode_arguments(self):
        module = self._load_node_module()
        runner, calls = self._runner_with_calls(module)
        runner.start("/tmp/legacy.launch", ["navigation_mode:=omni"])
        self.assertEqual(
            ["roslaunch", "/tmp/legacy.launch", "navigation_mode:=omni"],
            calls[-1],
        )

    def test_runner_without_arguments_keeps_baseline_command(self):
        module = self._load_node_module()
        runner, calls = self._runner_with_calls(module)
        runner.start("/tmp/stop.launch")
        self.assertEqual(["roslaunch", "/tmp/stop.launch"], calls[-1])

    def test_supervisor_config_reads_mode_params(self):
        module = self._load_node_module()
        module.rospy.get_param = lambda key, default: default
        node = module.NavigationHandoffSupervisorNode.__new__(
            module.NavigationHandoffSupervisorNode
        )
        config = node._read_supervisor_config()
        self.assertEqual("", config["legacy_nav_mode"])
        self.assertEqual("", config["stop_integration_nav_mode"])


if __name__ == "__main__":
    unittest.main()
