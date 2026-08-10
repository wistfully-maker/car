import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from stop_integration.mission_gate import MissionGate


def _install_stubs():
    for name in ("rospy", "std_msgs", "std_msgs.msg"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["std_msgs.msg"].String = type("String", (), {})


def load_module():
    _install_stubs()
    spec = importlib.util.spec_from_file_location(
        "stop_protocol_adapter_node_test",
        ROOT / "scripts" / "stop_protocol_adapter_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class String:
    def __init__(self, data=""):
        self.data = data


PHYSICAL_GOAL = {
    "protocol_version": 1,
    "task_id": "task-1",
    "goal_id": "delivery-1",
    "target_workshop": "食品加工车间",
    "selected_item": "苹果",
}
SIMULATION_GOAL = {
    "protocol_version": 1,
    "task_id": "task-1",
    "goal_id": "simulation-1",
    "target_workshop": "日用品加工车间",
    "selected_item": "毛巾",
}


def install_node(param_values=None):
    module = load_module()
    state = types.SimpleNamespace(
        messages=[],
        publishers={},
        subscribers={},
        logs=[],
        errors=[],
        now=10.0,
    )
    rospy = types.ModuleType("rospy")
    rospy.get_time = lambda: state.now
    rospy.Duration = lambda value: value
    rospy.sleep = lambda _value: None

    def publisher(topic, _kind, **_kwargs):
        def publish(message):
            state.messages.append((topic, json.loads(message.data)))
        state.publishers[topic] = publish
        return types.SimpleNamespace(publish=publish)

    rospy.Publisher = publisher
    rospy.Subscriber = lambda topic, _kind, callback, **_kw: (
        state.subscribers.__setitem__(topic, callback)
    )
    rospy.Timer = lambda _duration, callback: types.SimpleNamespace()
    rospy.logwarn = lambda *args: state.logs.append(args)
    rospy.logerr = lambda *args: state.errors.append(args)
    rospy.loginfo = lambda *args: None
    rospy.on_shutdown = lambda callback: None
    rospy.init_node = lambda *_args: None
    rospy.spin = lambda: None

    modules = {
        "rospy": rospy,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    modules["std_msgs.msg"].String = String
    sys.modules.update(modules)
    spec = importlib.util.spec_from_file_location(
        "stop_protocol_adapter_node_wiring_test",
        ROOT / "scripts" / "stop_protocol_adapter_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.StopProtocolAdapterNode()
    return node, state, module


def arrivals(state):
    return [
        payload
        for topic, payload in state.messages
        if topic in ("/task/delivery_arrived", "/task/simulation_arrived")
    ]


def acks(state):
    return [
        payload for topic, payload in state.messages if topic == "/stop/mission_ack"
    ]


class AdapterHappyPathTests(unittest.TestCase):
    def test_exact_correlated_arrival_outputs(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        self.assertEqual(
            [
                {
                    "protocol_version": 1,
                    "task_id": "task-1",
                    "goal_id": "delivery-1",
                    "status": "arrived",
                    "message": "",
                }
            ],
            arrivals(state),
        )
        state.subscribers["/task/simulation_navigation_goal"](
            String(data=json.dumps(SIMULATION_GOAL, ensure_ascii=False))
        )
        self.assertEqual(
            [
                {
                    "protocol_version": 1,
                    "task_id": "task-1",
                    "goal_id": "simulation-1",
                    "action": "start_phase2",
                    "simulation": {
                        "target_workshop": "日用品加工车间",
                        "selected_item": "毛巾",
                    },
                }
            ],
            acks(state),
        )
        state.subscribers["/stop/mission_event"](String(data="done"))
        self.assertEqual(
            [
                {
                    "protocol_version": 1,
                    "task_id": "task-1",
                    "goal_id": "delivery-1",
                    "status": "arrived",
                    "message": "",
                },
                {
                    "protocol_version": 1,
                    "task_id": "task-1",
                    "goal_id": "simulation-1",
                    "status": "arrived",
                    "message": "",
                },
            ],
            arrivals(state),
        )

    def test_physical_arrival_only_after_phase1_done(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        self.assertEqual([], arrivals(state))
        state.subscribers["/stop/mission_event"](String(data="done"))
        self.assertEqual([], arrivals(state))

    def test_simulation_arrival_only_after_second_parking(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        state.subscribers["/task/simulation_navigation_goal"](
            String(data=json.dumps(SIMULATION_GOAL, ensure_ascii=False))
        )
        self.assertEqual(1, len(arrivals(state)))
        self.assertEqual("/task/delivery_arrived",
                         [t for t, _p in state.messages][0])

    def test_ack_requires_matching_active_mission(self):
        _node, state, _module = install_node()
        state.subscribers["/task/simulation_navigation_goal"](
            String(data=json.dumps(SIMULATION_GOAL, ensure_ascii=False))
        )
        self.assertEqual([], acks(state))


class AdapterFailureTests(unittest.TestCase):
    def _phase2_failed(self, event):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        state.subscribers["/task/simulation_navigation_goal"](
            String(data=json.dumps(SIMULATION_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data=event))
        return state

    def test_phase2_failure_publishes_failed_simulation_arrival(self):
        state = self._phase2_failed("failed:not_found")
        self.assertEqual(
            [
                {
                    "protocol_version": 1,
                    "task_id": "task-1",
                    "goal_id": "simulation-1",
                    "status": "failed",
                    "message": "not_found",
                }
            ],
            arrivals(state)[1:],
        )

    def test_phase1_failure_publishes_failed_delivery_arrival(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="failed:cancelled"))
        self.assertEqual(
            [
                {
                    "protocol_version": 1,
                    "task_id": "task-1",
                    "goal_id": "delivery-1",
                    "status": "failed",
                    "message": "cancelled",
                }
            ],
            arrivals(state),
        )

    def test_failure_never_publishes_success(self):
        state = self._phase2_failed("failed:phase2_timeout")
        simulation = [
            a for a in arrivals(state) if a["goal_id"] == "simulation-1"
        ]
        self.assertTrue(simulation)
        self.assertEqual("failed", simulation[-1]["status"])
        self.assertNotIn("arrived", [a["status"] for a in simulation])


class AdapterDedupeTests(unittest.TestCase):
    def test_duplicate_terminal_event_republishes_cached_output(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        state.subscribers["/task/simulation_navigation_goal"](
            String(data=json.dumps(SIMULATION_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="done"))
        state.subscribers["/stop/mission_event"](String(data="done"))
        # 第一次 done 发布仿真到达；重复终态事件只重发缓存结果。
        self.assertEqual(3, len(arrivals(state)))
        self.assertEqual("simulation-1", arrivals(state)[-1]["goal_id"])

    def test_duplicate_phase1_done_publishes_delivery_once_then_cached(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        delivery = [
            p for t, p in state.messages if t == "/task/delivery_arrived"
        ]
        self.assertEqual(2, len(delivery))


class AdapterValidationTests(unittest.TestCase):
    def test_events_without_active_mission_are_ignored(self):
        _node, state, _module = install_node()
        state.subscribers["/stop/mission_event"](String(data="phase1_done"))
        state.subscribers["/stop/mission_event"](String(data="done"))
        self.assertEqual([], arrivals(state))

    def test_invalid_physical_goal_is_ignored(self):
        _node, state, _module = install_node()
        for raw in ("not json", json.dumps({"protocol_version": 2})):
            state.subscribers["/task/stop_mission_goal"](String(data=raw))
        self.assertEqual([], arrivals(state))

    def test_wrong_task_simulation_goal_is_ignored(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        wrong = dict(SIMULATION_GOAL, task_id="task-2")
        state.subscribers["/task/simulation_navigation_goal"](
            String(data=json.dumps(wrong, ensure_ascii=False))
        )
        self.assertEqual([], acks(state))

    def test_unknown_event_string_is_ignored(self):
        _node, state, _module = install_node()
        state.subscribers["/task/stop_mission_goal"](
            String(data=json.dumps(PHYSICAL_GOAL, ensure_ascii=False))
        )
        state.subscribers["/stop/mission_event"](String(data="mystery"))
        self.assertEqual([], arrivals(state))

    def test_node_declares_only_protocol_public_topics(self):
        _node, state, _module = install_node()
        self.assertEqual(
            {
                "/task/delivery_arrived",
                "/task/simulation_arrived",
                "/stop/mission_ack",
            },
            set(state.publishers),
        )
        self.assertEqual(
            {
                "/task/stop_mission_goal",
                "/task/simulation_navigation_goal",
                "/stop/mission_event",
            },
            set(state.subscribers),
        )


if __name__ == "__main__":
    unittest.main()
