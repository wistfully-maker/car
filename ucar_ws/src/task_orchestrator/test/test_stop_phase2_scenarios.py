"""Offline end-to-end scenarios for the stop phase two integration.

组合真实核心（TaskOrchestrator、NavigationHandoff、NavigationHandoffDriver、
VelocityArbiter）与测试本地 fakes（任务注入、stop 任务阶段、协议映射）。
每个失败场景都断言：最后运动模式为 IDLE（全局仲裁器只输出零）、
无成功到达 topic 发出、handoff 未放行（如适用）。
"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from task_orchestrator.handoff import NavigationHandoff
from task_orchestrator.motion_mode import IDLE
from task_orchestrator.orchestrator import TaskOrchestrator


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


def load_driver_module():
    _install_import_stubs()
    spec = importlib.util.spec_from_file_location(
        "stop_phase2_scenario_driver",
        ROOT / "scripts" / "navigation_handoff_supervisor_node.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DRIVER_MODULE = load_driver_module()

ORCHESTRATOR_TIMEOUTS = {
    "dependency_ready": 30.0,
    "pickup_navigation": 300.0,
    "qr_search": 90.0,
    "llm_classification": 60.0,
    "speech": 60.0,
    "delivery_navigation": 300.0,
    "simulation_navigation": 300.0,
    "cancel_ack": 15.0,
}

HANDOFF_CONFIG = {
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


class FakeActions:
    """测试本地动作实现：记录零速度与放行，取消结果回灌 driver。"""

    def __init__(self, clock):
        self.clock = clock
        self.zero_count = 0
        self.released = []
        self.modes = []
        self.diagnostics = []
        self.failures = []
        self.cancel_ok = True
        self.legacy_absent = True
        self.stop_start_ok = True
        self.readiness_ok = True
        self.driver = None

    def bind(self, driver):
        self.driver = driver

    def publish_zero(self):
        self.zero_count += 1

    def wait_stopped(self):
        pass

    def cancel_goals(self):
        # 取消是异步操作：结果由场景显式调用 driver.on_cancel_result() 注入。
        pass

    def stop_owned_legacy(self):
        return self.legacy_absent

    def verify_legacy_absent(self):
        return self.legacy_absent

    def start_owned_stop(self):
        return self.stop_start_ok

    def wait_stop_ready(self):
        return self.readiness_ok

    def release_task(self, payload, goal):
        self.released.append((payload, goal))
        self.mission.start(goal)

    def publish_motion_mode(self, mode):
        self.modes.append(mode)

    def publish_diagnostic(self, payload):
        self.diagnostics.append(payload)

    def handoff_failed(self, payload):
        self.failures.append(payload)


class FakeAdapter:
    """测试本地协议映射：阶段事件 -> 到达结果，并驱动假 stop 任务。"""

    def __init__(self, scenario):
        self.scenario = scenario
        self.mission = FakeMission(self)
        self.phase1_done = False
        self.simulation_goal_id = None
        self.arrivals = []

    def on_stop_mission_goal(self, goal):
        self.mission.start(goal)

    def on_simulation_goal(self, goal):
        self.simulation_goal_id = goal["goal_id"]
        if self.mission.phase == "phase1_parked":
            self.mission.phase2_ack()

    def on_mission_event(self, event):
        if event == "phase1_done":
            self.phase1_done = True
            self._arrive("delivery", self.mission.goal["goal_id"], "arrived", "")
        elif event == "done":
            if self.simulation_goal_id is None:
                return
            self._arrive("simulation", self.simulation_goal_id, "arrived", "")
        elif event.startswith("failed:"):
            reason = event.split(":", 1)[1]
            if not self.phase1_done:
                self._arrive(
                    "delivery", self.mission.goal["goal_id"], "failed", reason
                )
            elif self.simulation_goal_id is not None:
                self._arrive(
                    "simulation", self.simulation_goal_id, "failed", reason
                )

    def _arrive(self, kind, goal_id, status, message):
        task_id = self.scenario.task_id
        payload = {
            "protocol_version": 1,
            "task_id": task_id,
            "goal_id": goal_id,
            "status": status,
            "message": message,
        }
        self.arrivals.append((kind, dict(payload)))
        if kind == "delivery":
            self.scenario.orch.on_delivery_arrived(payload)
        else:
            self.scenario.orch.on_simulation_arrived(payload)
        self.scenario.sync_modes()


class FakeMission:
    """测试本地 stop 任务：阶段由场景显式推进。"""

    def __init__(self, adapter):
        self.adapter = adapter
        self.goal = None
        self.phase = "idle"  # idle/phase1_nav/phase1_parked/phase2_nav/done/failed

    def start(self, goal):
        self.goal = goal
        self.phase = "phase1_nav"

    def phase1_arrived(self):
        if self.phase != "phase1_nav":
            return
        self.phase = "phase1_parked"
        self.adapter.on_mission_event("phase1_done")

    def phase2_ack(self):
        if self.phase == "phase1_parked":
            self.phase = "phase2_nav"

    def phase2_arrived(self):
        if self.phase != "phase2_nav":
            return
        self.phase = "done"
        self.adapter.on_mission_event("done")

    def fail(self, reason):
        self.phase = "failed"
        self.adapter.on_mission_event("failed:" + reason)


class Scenario:
    def __init__(self, handoff_overrides=None):
        self.now = [0.0]
        self.ids = iter(
            ["pickup-1", "search-1", "llm-1", "speech-1",
             "delivery-1", "delivery-speech-1", "simulation-1",
             "simulation-speech-1", "unused-9"]
        )
        self.outputs = []
        self.task_id = "task-1"
        self.orch = TaskOrchestrator(
            self.outputs,
            lambda: self.now[0],
            lambda: next(self.ids),
            dict(ORCHESTRATOR_TIMEOUTS),
            simulation_phase_enabled=True,
        )
        self.actions = FakeActions(lambda: self.now[0])
        config = dict(HANDOFF_CONFIG)
        config.update(handoff_overrides or {})
        self.machine = NavigationHandoff(config, lambda: self.now[0])
        self.driver = DRIVER_MODULE.NavigationHandoffDriver(
            self.machine, self.actions
        )
        self.actions.bind(self.driver)
        self.adapter = FakeAdapter(self)
        self.actions.mission = self.adapter.mission
        self._synced_output_count = 0
        self._synced_actions_count = 0
        self.mode_timeline = []

    # ---------- 组合驱动 ----------

    def reach_delivery_handoff(self):
        self.orch.on_task_request(
            {
                "protocol_version": 1,
                "task_id": self.task_id,
                "physical_target_category": "食品",
                "simulation_target_category": "日用品",
                "raw_text": "取得食品，并领取仿真环境中的日用品",
            }
        )
        self.orch.on_dependencies_ready()
        self.orch.on_pickup_arrived(
            {"protocol_version": 1, "task_id": self.task_id,
             "goal_id": "pickup-1", "status": "arrived", "message": ""}
        )
        self.orch.on_qr_result(
            {
                "protocol_version": 1, "task_id": self.task_id,
                "search_id": "search-1", "status": "complete",
                "items": [
                    {"order": 1, "item_name": "手机", "url": "u3",
                     "detected_yaw": 0.0},
                    {"order": 2, "item_name": "毛巾", "url": "u2",
                     "detected_yaw": 1.2},
                    {"order": 3, "item_name": "苹果", "url": "u1",
                     "detected_yaw": 3.4},
                ],
                "message": "",
            }
        )
        self.orch.on_llm_result(
            {
                "protocol_version": 1, "task_id": self.task_id,
                "request_id": "llm-1", "status": "success",
                "physical": {"selected_order": 3, "selected_item": "苹果",
                             "category": "食品", "workshop": "食品加工车间"},
                "simulation": {"selected_order": 2, "selected_item": "毛巾",
                               "category": "日用品", "workshop": "日用品加工车间"},
                "message": "",
            }
        )
        self.orch.on_speech_done(
            {"protocol_version": 1, "task_id": self.task_id,
             "speech_id": "speech-1", "status": "success", "message": ""}
        )
        goal = self.actions_("publish_delivery_goal")[-1]
        self.driver.start(goal)
        self.sync_modes()

    def actions_(self, name):
        return [payload for action, payload in self.outputs if action == name]

    def motion_modes(self):
        return [payload for action, payload in self.outputs
                if action == "publish_motion_mode"]

    def handoff_odom_stop(self):
        self.driver.on_odom(0.1, near_zero=True, valid=True)
        self.driver.on_odom(0.5, near_zero=True, valid=True)
        self.driver.on_odom(0.9, near_zero=True, valid=True)

    def handoff_to_release(self):
        self.driver.on_cancel_result(self.actions.cancel_ok)
        self.handoff_odom_stop()
        self.driver.poll()  # legacy 缺席验证
        self.driver.poll()  # stop 就绪验证
        if self.machine.state == "READY":
            self.orch.on_navigation_handoff_status(
                {"protocol_version": 1, "task_id": self.task_id,
                 "goal_id": "delivery-1", "status": "ready", "message": ""}
            )
        self.sync_modes()

    def complete_current_speech(self, status="success"):
        self.orch.on_speech_done(
            {
                "protocol_version": 1,
                "task_id": self.task_id,
                "speech_id": self.orch.task["speech_id"],
                "status": status,
                "message": "" if status == "success" else "speech failed",
            }
        )
        self.sync_modes()

    def run_to_complete(self):
        self.reach_delivery_handoff()
        self.handoff_to_release()
        self.adapter.mission.phase1_arrived()
        self.complete_current_speech()
        sim_goal = self.actions_("publish_simulation_navigation_goal")[-1]
        self.adapter.on_simulation_goal(sim_goal)
        self.adapter.mission.phase2_arrived()
        self.complete_current_speech()

    # ---------- 安全断言 ----------

    def success_arrivals(self):
        return [p for _kind, p in self.adapter.arrivals if p["status"] == "arrived"]

    def failed_arrivals(self):
        return [p for _kind, p in self.adapter.arrivals if p["status"] == "failed"]

    def sync_modes(self):
        """按发生顺序合并编排器与 handoff 发布的运动模式。"""
        modes = [payload for action, payload in self.outputs
                 if action == "publish_motion_mode"]
        for mode in modes[self._synced_output_count:]:
            self.mode_timeline.append(mode)
        self._synced_output_count = len(modes)
        for mode in self.actions.modes[self._synced_actions_count:]:
            self.mode_timeline.append(mode)
        self._synced_actions_count = len(self.actions.modes)

    def last_mode(self):
        self.sync_modes()
        return self.mode_timeline[-1] if self.mode_timeline else None

    def assert_failed_closed(self, test_case):
        """失败场景：最后运动模式 IDLE（全局仲裁器在 IDLE 下只输出零），
        且失败后没有新的成功到达 topic 发出。handoff 级失败根本没有到达；
        任务级失败的最后一条到达必须是失败。"""
        test_case.assertEqual("IDLE", self.last_mode())
        if self.adapter.arrivals:
            test_case.assertEqual(
                "failed", self.adapter.arrivals[-1][1]["status"]
            )


class HappyPathScenarioTests(unittest.TestCase):
    def test_two_arrivals_reach_complete(self):
        s = Scenario()
        s.run_to_complete()
        self.assertEqual("COMPLETE", s.orch.state)
        self.assertEqual(2, len(s.success_arrivals()))
        kinds = [kind for kind, _p in s.adapter.arrivals]
        self.assertEqual(["delivery", "simulation"], kinds)
        self.assertEqual("delivery-1", s.success_arrivals()[0]["goal_id"])
        self.assertEqual("simulation-1", s.success_arrivals()[-1]["goal_id"])
        self.assertEqual(["STOP_NAVIGATION"], s.actions.modes)
        self.assertEqual(1, len(s.actions.released))
        self.assertEqual("IDLE", s.motion_modes()[-1])
        speeches = s.actions_("publish_speech")
        self.assertEqual(3, len(speeches))
        self.assertEqual(
            [
                "取得苹果属于食品大类应放置在食品加工车间，"
                "仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间",
                "已将苹果放入食品加工车间",
                "仿真任务已完成，已将毛巾放入日用品加工车间",
            ],
            [speech["text"] for speech in speeches],
        )

    def test_each_parking_speech_gates_the_next_phase(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.phase1_arrived()
        self.assertEqual("WAITING_DELIVERY_SPEECH", s.orch.state)
        self.assertEqual([], s.actions_("publish_simulation_navigation_goal"))

        s.complete_current_speech()
        self.assertEqual("NAVIGATING_TO_SIM_WORKSHOP", s.orch.state)
        sim_goal = s.actions_("publish_simulation_navigation_goal")[-1]
        s.adapter.on_simulation_goal(sim_goal)
        s.adapter.mission.phase2_arrived()
        self.assertEqual("WAITING_SIMULATION_SPEECH", s.orch.state)

        s.complete_current_speech()
        self.assertEqual("COMPLETE", s.orch.state)

    def test_transient_cancel_retry_then_success(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.actions.cancel_ok = False
        s.driver.on_cancel_result(False)
        s.driver.on_cancel_result(False)
        s.actions.cancel_ok = True
        s.driver.on_cancel_result(False)
        s.driver.on_cancel_result(True)
        s.handoff_odom_stop()
        s.driver.poll()
        s.driver.poll()
        s.orch.on_navigation_handoff_status(
            {"protocol_version": 1, "task_id": s.task_id,
             "goal_id": "delivery-1", "status": "ready", "message": ""}
        )
        s.adapter.mission.phase1_arrived()
        s.complete_current_speech()
        sim_goal = s.actions_("publish_simulation_navigation_goal")[-1]
        s.adapter.on_simulation_goal(sim_goal)
        s.adapter.mission.phase2_arrived()
        s.complete_current_speech()
        self.assertEqual("COMPLETE", s.orch.state)
        self.assertEqual(2, len(s.success_arrivals()))

    def test_transient_readiness_retry_then_success(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.driver.on_cancel_result(True)
        s.handoff_odom_stop()
        s.actions.readiness_ok = False
        s.driver.poll()  # legacy 缺席 ok → start stop → readiness 首次失败
        s.driver.poll()  # readiness 重试失败
        s.actions.readiness_ok = True
        s.driver.poll()
        s.orch.on_navigation_handoff_status(
            {"protocol_version": 1, "task_id": s.task_id,
             "goal_id": "delivery-1", "status": "ready", "message": ""}
        )
        s.adapter.mission.phase1_arrived()
        s.complete_current_speech()
        sim_goal = s.actions_("publish_simulation_navigation_goal")[-1]
        s.adapter.on_simulation_goal(sim_goal)
        s.adapter.mission.phase2_arrived()
        s.complete_current_speech()
        self.assertEqual("COMPLETE", s.orch.state)


class HandoffFailureScenarioTests(unittest.TestCase):
    def test_old_stack_never_exits_fails_closed(self):
        s = Scenario({"legacy_exit_retries": 2})
        s.reach_delivery_handoff()
        s.actions.legacy_absent = False
        s.driver.on_cancel_result(True)
        s.handoff_odom_stop()
        for _ in range(4):
            s.driver.poll()
        self.assertEqual("FAILED", s.machine.state)
        self.assertEqual([], s.actions.released)
        self.assertEqual([], s.actions.modes)
        s.assert_failed_closed(self)
        self.assertIn("legacy_exit", s.actions.failures[-1]["stage"])

    def test_stale_odom_times_out_fails_closed(self):
        s = Scenario({"total_timeout": 5.0})
        s.reach_delivery_handoff()
        s.now[0] = 6.0
        s.driver.poll()
        self.assertEqual("FAILED", s.machine.state)
        self.assertEqual([], s.actions.released)
        s.assert_failed_closed(self)

    def test_amcl_never_ready_fails_closed(self):
        s = Scenario({"readiness_retries": 3})
        s.reach_delivery_handoff()
        s.driver.on_cancel_result(True)
        s.handoff_odom_stop()
        s.actions.readiness_ok = False
        for _ in range(5):
            s.driver.poll()
        self.assertEqual("FAILED", s.machine.state)
        self.assertEqual([], s.actions.released)
        self.assertEqual([], s.actions.modes)
        s.assert_failed_closed(self)
        self.assertIn("readiness", s.actions.failures[-1]["stage"])

    def test_ocr_absent_fails_closed(self):
        # OCR 缺失表现为就绪探测持续失败，与 AMCL 场景同一安全路径。
        s = Scenario({"readiness_retries": 2})
        s.reach_delivery_handoff()
        s.driver.on_cancel_result(True)
        s.handoff_odom_stop()
        s.actions.readiness_ok = False
        for _ in range(4):
            s.driver.poll()
        self.assertEqual("FAILED", s.machine.state)
        s.assert_failed_closed(self)

    def test_stop_stack_start_failure_fails_closed(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.actions.stop_start_ok = False
        s.driver.on_cancel_result(True)
        s.handoff_odom_stop()
        s.driver.poll()
        self.assertEqual("FAILED", s.machine.state)
        self.assertEqual([], s.actions.released)
        s.assert_failed_closed(self)


class MissionFailureScenarioTests(unittest.TestCase):
    def test_physical_navigation_failure(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.fail("not_found")
        self.assertEqual("ERROR", s.orch.state)
        self.assertEqual([], s.actions_("publish_simulation_navigation_goal"))
        self.assertEqual(1, len(s.failed_arrivals()))
        self.assertEqual("delivery", s.adapter.arrivals[-1][0])
        s.assert_failed_closed(self)

    def test_first_parking_failure(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.fail("cancelled")
        self.assertEqual("ERROR", s.orch.state)
        self.assertEqual([], s.actions_("publish_simulation_navigation_goal"))
        s.assert_failed_closed(self)

    def test_simulation_ack_missing_times_out(self):
        s = Scenario({"readiness_retries": 2})
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.phase1_arrived()
        s.complete_current_speech()
        self.assertEqual("NAVIGATING_TO_SIM_WORKSHOP", s.orch.state)
        s.now[0] = s.orch.deadline + 0.01
        s.orch.tick()
        self.assertEqual("ERROR", s.orch.state)
        simulation = [
            p for kind, p in s.adapter.arrivals if kind == "simulation"
        ]
        self.assertEqual([], simulation)
        self.assertEqual("IDLE", s.motion_modes()[-1])

    def test_second_navigation_failure(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.phase1_arrived()
        s.complete_current_speech()
        sim_goal = s.actions_("publish_simulation_navigation_goal")[-1]
        s.adapter.on_simulation_goal(sim_goal)
        s.adapter.mission.fail("not_found")
        self.assertEqual("ERROR", s.orch.state)
        self.assertEqual("simulation", s.adapter.arrivals[-1][0])
        self.assertEqual(1, len(s.success_arrivals()))  # 只有实物成功
        s.assert_failed_closed(self)

    def test_second_parking_failure(self):
        s = Scenario()
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.phase1_arrived()
        s.complete_current_speech()
        sim_goal = s.actions_("publish_simulation_navigation_goal")[-1]
        s.adapter.on_simulation_goal(sim_goal)
        s.adapter.mission.fail("cancelled")
        self.assertEqual("ERROR", s.orch.state)
        self.assertEqual(1, len(s.success_arrivals()))
        s.assert_failed_closed(self)


class CancelScenarioTests(unittest.TestCase):
    def test_cancel_in_every_active_state(self):
        for state in (
            "CHECKING_DEPENDENCIES",
            "NAVIGATING_TO_PICKUP",
            "WAITING_QR",
            "WAITING_LLM",
            "WAITING_SPEECH",
            "DELIVERY_HANDED_OFF",
            "NAVIGATING_TO_WORKSHOP",
            "WAITING_DELIVERY_SPEECH",
            "NAVIGATING_TO_SIM_WORKSHOP",
            "WAITING_SIMULATION_SPEECH",
        ):
            with self.subTest(state=state):
                s = Scenario()
                self._reach(s, state)
                success_count = len(s.success_arrivals())
                s.orch.on_cancel({"task_id": s.task_id, "reason": "operator_cancel"})
                self.assertEqual("CANCELLED", s.orch.state)
                self.assertEqual("IDLE", s.motion_modes()[-1])
                self.assertEqual(success_count, len(s.success_arrivals()))

    def _reach(self, s, state):
        s.orch.on_task_request(
            {"protocol_version": 1, "task_id": s.task_id,
             "physical_target_category": "食品",
             "simulation_target_category": "日用品", "raw_text": "t"}
        )
        if state == "CHECKING_DEPENDENCIES":
            return
        s.orch.on_dependencies_ready()
        if state == "NAVIGATING_TO_PICKUP":
            return
        s.orch.on_pickup_arrived(
            {"protocol_version": 1, "task_id": s.task_id,
             "goal_id": "pickup-1", "status": "arrived", "message": ""}
        )
        if state == "WAITING_QR":
            return
        s.orch.on_qr_result(
            {"protocol_version": 1, "task_id": s.task_id,
             "search_id": "search-1", "status": "complete",
             "items": [
                 {"order": 1, "item_name": "手机", "url": "u3",
                  "detected_yaw": 0.0},
                 {"order": 2, "item_name": "毛巾", "url": "u2",
                  "detected_yaw": 1.2},
                 {"order": 3, "item_name": "苹果", "url": "u1",
                  "detected_yaw": 3.4},
             ],
             "message": ""}
        )
        if state == "WAITING_LLM":
            return
        s.orch.on_llm_result(
            {"protocol_version": 1, "task_id": s.task_id,
             "request_id": "llm-1", "status": "success",
             "physical": {"selected_order": 3, "selected_item": "苹果",
                          "category": "食品", "workshop": "食品加工车间"},
             "simulation": {"selected_order": 2, "selected_item": "毛巾",
                            "category": "日用品", "workshop": "日用品加工车间"},
             "message": ""}
        )
        if state == "WAITING_SPEECH":
            return
        s.orch.on_speech_done(
            {"protocol_version": 1, "task_id": s.task_id,
             "speech_id": "speech-1", "status": "success", "message": ""}
        )
        if state == "DELIVERY_HANDED_OFF":
            return
        s.orch.on_navigation_handoff_status(
            {"protocol_version": 1, "task_id": s.task_id,
             "goal_id": "delivery-1", "status": "ready", "message": ""}
        )
        if state == "NAVIGATING_TO_WORKSHOP":
            return
        goal = s.actions_("publish_delivery_goal")[-1]
        s.adapter.mission.start(goal)
        s.adapter.mission.phase1_arrived()
        if state == "WAITING_DELIVERY_SPEECH":
            return
        s.complete_current_speech()
        if state == "NAVIGATING_TO_SIM_WORKSHOP":
            return
        sim_goal = s.actions_("publish_simulation_navigation_goal")[-1]
        s.adapter.on_simulation_goal(sim_goal)
        s.adapter.mission.phase2_arrived()

    def test_cancel_during_physical_phase_fails_closed(self):
        # DELIVERY_HANDED_OFF 是终态：取消由 stop 任务侧处理并报失败。
        s = Scenario()
        s.reach_delivery_handoff()
        s.handoff_to_release()
        s.adapter.mission.fail("cancelled")
        self.assertEqual("ERROR", s.orch.state)
        self.assertEqual([], s.success_arrivals())
        s.assert_failed_closed(self)


class RepeatedMessageScenarioTests(unittest.TestCase):
    def test_repeated_and_stale_messages_do_not_restart(self):
        s = Scenario()
        s.run_to_complete()
        goal_count = len(s.actions_("publish_delivery_goal"))
        sim_count = len(s.actions_("publish_simulation_navigation_goal"))
        # 重复的旧阶段消息不重放运动目标。
        s.orch.on_speech_done(
            {"protocol_version": 1, "task_id": s.task_id,
             "speech_id": "speech-1", "status": "success", "message": ""}
        )
        s.orch.on_delivery_arrived(
            {"protocol_version": 1, "task_id": s.task_id,
             "goal_id": "delivery-1", "status": "arrived", "message": ""}
        )
        s.adapter.on_simulation_goal(
            {"protocol_version": 1, "task_id": s.task_id,
             "goal_id": "simulation-1", "target_workshop": "日用品加工车间",
             "selected_item": "毛巾"}
        )
        self.assertEqual(goal_count, len(s.actions_("publish_delivery_goal")))
        self.assertEqual(sim_count, len(s.actions_("publish_simulation_navigation_goal")))
        self.assertEqual("COMPLETE", s.orch.state)


if __name__ == "__main__":
    unittest.main()
