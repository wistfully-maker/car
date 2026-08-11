import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.orchestrator import TaskOrchestrator


TIMEOUTS = {
    "dependency_ready": 30.0,
    "pickup_navigation": 300.0,
    "qr_search": 90.0,
    "llm_classification": 60.0,
    "speech": 60.0,
    "delivery_navigation": 300.0,
    "simulation_navigation": 300.0,
    "cancel_ack": 15.0,
}


class Harness:
    def __init__(self, simulation_phase_enabled=False):
        self.outputs = []
        self.now = [1000.0]
        self.ids = iter(
            [
                "pickup-1",
                "search-1",
                "llm-1",
                "speech-1",
                "delivery-1",
                "delivery-speech-1",
                "simulation-1",
                "simulation-speech-1",
                "unused-9",
            ]
        )
        self.orch = TaskOrchestrator(
            self.outputs,
            lambda: self.now[0],
            lambda: next(self.ids),
            dict(TIMEOUTS),
            simulation_phase_enabled=simulation_phase_enabled,
        )

    def task_request(self, task_id="task-1"):
        self.orch.on_task_request(
            {
                "protocol_version": 1,
                "task_id": task_id,
                "physical_target_category": "食品",
                "simulation_target_category": "日用品",
                "raw_text": "取得食品，并领取仿真环境中的日用品",
            }
        )

    def dependencies_ready(self):
        self.orch.on_dependencies_ready()

    def pickup_arrived(self, status="arrived"):
        self.orch.on_pickup_arrived(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": "pickup-1",
                "status": status,
                "message": "" if status == "arrived" else "navigation failed",
            }
        )

    def qr_result(self, status="complete", search_id="search-1"):
        items = [
            {
                "order": 1,
                "item_name": "手机",
                "url": "https://example/3",
                "detected_yaw": 0.0,
            },
            {
                "order": 2,
                "item_name": "毛巾",
                "url": "https://example/2",
                "detected_yaw": 1.2,
            },
            {
                "order": 3,
                "item_name": "苹果",
                "url": "https://example/1",
                "detected_yaw": 3.4,
            },
        ]
        self.orch.on_qr_result(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "search_id": search_id,
                "status": status,
                "items": items if status == "complete" else [],
                "message": "" if status == "complete" else "items not found",
            }
        )

    def llm_result(self, status="success", request_id="llm-1"):
        message = {
            "protocol_version": 1,
            "task_id": "task-1",
            "request_id": request_id,
            "status": status,
            "message": "" if status == "success" else "LLM failed",
        }
        if status == "success":
            message.update(
                physical={
                    "selected_order": 3,
                    "selected_item": "苹果",
                    "category": "食品",
                    "workshop": "食品加工车间",
                },
                simulation={
                    "selected_order": 2,
                    "selected_item": "毛巾",
                    "category": "日用品",
                    "workshop": "日用品加工车间",
                },
            )
        self.orch.on_llm_result(message)

    def speech_done(self, status="success", speech_id=None):
        if speech_id is None:
            speech_id = self.orch.task["speech_id"]
        self.orch.on_speech_done(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "speech_id": speech_id,
                "status": status,
                "message": "" if status == "success" else "TTS failed",
            }
        )

    def delivery_arrived(self, status="arrived"):
        self.orch.on_delivery_arrived(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": "delivery-1",
                "status": status,
                "message": "" if status == "arrived" else "navigation failed",
            }
        )

    def handoff_status(self, status="ready", goal_id="delivery-1"):
        self.orch.on_navigation_handoff_status(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": goal_id,
                "status": status,
                "message": "" if status == "ready" else "handoff failed",
            }
        )

    def simulation_arrived(self, goal_id="simulation-1", status="arrived"):
        self.orch.on_simulation_arrived(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": goal_id,
                "status": status,
                "message": "" if status == "arrived" else "navigation failed",
            }
        )

    def reach(self, state):
        self.task_request()
        if state == "CHECKING_DEPENDENCIES":
            return
        self.dependencies_ready()
        if state == "NAVIGATING_TO_PICKUP":
            return
        self.pickup_arrived()
        if state == "WAITING_QR":
            return
        self.qr_result()
        if state == "WAITING_LLM":
            return
        self.llm_result()
        if state == "WAITING_SPEECH":
            return
        self.speech_done()
        if state == "DELIVERY_HANDED_OFF":
            return
        self.handoff_status()
        if state == "NAVIGATING_TO_WORKSHOP":
            return
        self.delivery_arrived()
        if state == "WAITING_DELIVERY_SPEECH":
            return
        self.speech_done()
        if state == "NAVIGATING_TO_SIM_WORKSHOP":
            return
        self.simulation_arrived()
        if state == "WAITING_SIMULATION_SPEECH":
            return
        self.speech_done()

    def actions(self, name):
        return [payload for action, payload in self.outputs if action == name]


class OrchestratorHappyPathTests(unittest.TestCase):
    def test_every_transition_emits_motion_mode_and_handoff_is_idle(self):
        h = Harness()
        self.assertEqual(["IDLE"], h.actions("publish_motion_mode"))
        h.reach("DELIVERY_HANDED_OFF")
        self.assertEqual(
            ["IDLE", "IDLE", "NAVIGATION", "QR_SEARCH", "IDLE", "IDLE", "IDLE"],
            h.actions("publish_motion_mode"),
        )

    def test_motion_mode_action_precedes_status_for_every_transition(self):
        h = Harness()
        h.reach("DELIVERY_HANDED_OFF")
        transition_actions = [action for action, _payload in h.outputs]
        status_indexes = [i for i, action in enumerate(transition_actions) if action == "publish_status"]
        for status_index in status_indexes:
            self.assertEqual("publish_motion_mode", transition_actions[status_index - 1])

    def test_full_success_path_and_output_contracts(self):
        h = Harness()
        expected = (
            ("CHECKING_DEPENDENCIES", h.task_request),
            ("NAVIGATING_TO_PICKUP", h.dependencies_ready),
            ("WAITING_QR", h.pickup_arrived),
            ("WAITING_LLM", h.qr_result),
            ("WAITING_SPEECH", h.llm_result),
            ("DELIVERY_HANDED_OFF", h.speech_done),
        )
        for state, event in expected:
            event()
            self.assertEqual(state, h.orch.state)

        self.assertEqual("pickup-1", h.actions("publish_pickup_goal")[0]["goal_id"])
        self.assertEqual(3, h.actions("publish_qr_start")[0]["expected_count"])
        llm = h.actions("publish_llm_request")[0]
        self.assertEqual("食品", llm["physical_target_category"])
        self.assertEqual("日用品", llm["simulation_target_category"])
        self.assertEqual(
            ["手机", "毛巾", "苹果"],
            [item["item_name"] for item in llm["candidates"]],
        )
        self.assertEqual(
            "取得苹果属于食品大类应放置在食品加工车间，"
            "仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间",
            h.actions("publish_speech")[0]["text"],
        )
        delivery = h.actions("publish_delivery_goal")[0]
        self.assertEqual("食品加工车间", delivery["target_workshop"])
        self.assertEqual("苹果", delivery["selected_item"])
        self.assertNotIn("simulation", delivery)


class OrchestratorFailureTests(unittest.TestCase):
    def test_adapter_internal_error_uses_public_failure_api(self):
        h = Harness()
        h.reach("WAITING_LLM")
        h.orch.on_internal_error("callback crashed")
        self.assertEqual("ERROR", h.orch.state)
        self.assertEqual(
            "callback crashed",
            h.actions("publish_status")[-1]["message"],
        )

    def test_stage_failures_enter_error_without_downstream_action(self):
        cases = (
            ("NAVIGATING_TO_PICKUP", lambda h: h.pickup_arrived("failed")),
            ("WAITING_QR", lambda h: h.qr_result("not_found")),
            ("WAITING_LLM", lambda h: h.llm_result("error")),
            ("WAITING_SPEECH", lambda h: h.speech_done("error")),
            ("NAVIGATING_TO_WORKSHOP", lambda h: h.delivery_arrived("failed")),
        )
        for state, failure in cases:
            with self.subTest(state=state):
                h = Harness()
                h.reach(state)
                failure(h)
                self.assertEqual("ERROR", h.orch.state)
                self.assertEqual("IDLE", h.actions("publish_motion_mode")[-1])
                self.assertTrue(h.actions("publish_status")[-1]["message"])

    def test_every_active_stage_has_its_own_timeout(self):
        for state in (
            "CHECKING_DEPENDENCIES",
            "NAVIGATING_TO_PICKUP",
            "WAITING_QR",
            "WAITING_LLM",
            "WAITING_SPEECH",
            "WAITING_DELIVERY_SPEECH",
            "WAITING_SIMULATION_SPEECH",
        ):
            with self.subTest(state=state):
                h = Harness(simulation_phase_enabled=(
                    state in (
                        "WAITING_DELIVERY_SPEECH",
                        "WAITING_SIMULATION_SPEECH",
                    )
                ))
                h.reach(state)
                h.now[0] = h.orch.deadline + 0.01
                h.orch.tick()
                self.assertEqual("ERROR", h.orch.state)
                self.assertEqual("IDLE", h.actions("publish_motion_mode")[-1])
                status = h.actions("publish_status")[-1]
                self.assertIn(state, status["message"])

    def test_qr_timeout_and_cancel_emit_stop_once(self):
        for event in ("timeout", "cancel"):
            with self.subTest(event=event):
                h = Harness()
                h.reach("WAITING_QR")
                if event == "timeout":
                    h.now[0] = h.orch.deadline + 0.01
                    h.orch.tick()
                else:
                    h.orch.on_cancel(
                        {"task_id": "task-1", "reason": "operator_cancel"}
                    )
                stops = h.actions("publish_qr_stop")
                self.assertEqual(1, len(stops))
                self.assertEqual("search-1", stops[0]["search_id"])


class OrchestratorSafetyTests(unittest.TestCase):
    def test_duplicate_and_busy_task_requests_do_not_restart(self):
        h = Harness()
        h.task_request()
        deadline = h.orch.deadline
        h.task_request()
        self.assertEqual(deadline, h.orch.deadline)
        self.assertEqual("CHECKING_DEPENDENCIES", h.orch.state)

        h.task_request("task-2")
        self.assertEqual("task-1", h.orch.task["task_id"])
        self.assertEqual("busy", h.actions("publish_status")[-1]["status"])
        h.task_request()
        self.assertEqual("accepted", h.actions("publish_status")[-1]["status"])

    def test_stale_stage_identities_are_ignored(self):
        cases = (
            (
                "WAITING_QR",
                lambda h: h.qr_result(search_id="stale-search"),
            ),
            (
                "WAITING_LLM",
                lambda h: h.llm_result(request_id="stale-request"),
            ),
            (
                "WAITING_SPEECH",
                lambda h: h.speech_done(speech_id="stale-speech"),
            ),
        )
        for state, stale_event in cases:
            with self.subTest(state=state):
                h = Harness()
                h.reach(state)
                output_count = len(h.outputs)
                stale_event(h)
                self.assertEqual(state, h.orch.state)
                self.assertEqual(output_count, len(h.outputs))

    def test_repeated_completed_stage_message_does_not_publish_twice(self):
        h = Harness()
        h.reach("WAITING_LLM")
        llm_count = len(h.actions("publish_llm_request"))
        status_count = len(h.actions("publish_status"))
        h.qr_result()
        self.assertEqual("WAITING_LLM", h.orch.state)
        self.assertEqual(llm_count, len(h.actions("publish_llm_request")))
        self.assertEqual(status_count + 1, len(h.actions("publish_status")))

    def test_cancel_works_in_every_active_state(self):
        for state in (
            "CHECKING_DEPENDENCIES",
            "NAVIGATING_TO_PICKUP",
            "WAITING_QR",
            "WAITING_LLM",
            "WAITING_SPEECH",
            "WAITING_DELIVERY_SPEECH",
            "WAITING_SIMULATION_SPEECH",
        ):
            with self.subTest(state=state):
                h = Harness(simulation_phase_enabled=(
                    state in (
                        "WAITING_DELIVERY_SPEECH",
                        "WAITING_SIMULATION_SPEECH",
                    )
                ))
                h.reach(state)
                h.orch.on_cancel(
                    {"task_id": "task-1", "reason": "operator_cancel"}
                )
                self.assertEqual("CANCELLED", h.orch.state)
                self.assertEqual("IDLE", h.actions("publish_motion_mode")[-1])
                self.assertEqual(
                    "cancelled",
                    h.actions("publish_status")[-1]["status"],
                )

    def test_terminal_state_accepts_a_new_task(self):
        h = Harness()
        h.reach("NAVIGATING_TO_WORKSHOP")
        output_count = len(h.outputs)
        h.task_request()
        self.assertEqual("NAVIGATING_TO_WORKSHOP", h.orch.state)
        self.assertEqual(output_count + 1, len(h.outputs))

        h.orch.on_task_request(
            {
                "task_id": "task-2",
                "physical_target_category": "电子产品",
                "simulation_target_category": "食品",
                "raw_text": "new task",
            }
        )
        self.assertEqual("NAVIGATING_TO_WORKSHOP", h.orch.state)
        self.assertEqual("task-1", h.orch.task["task_id"])


class OrchestratorDualWorkshopTests(unittest.TestCase):
    """Lock the correlated dual-workshop arrival contract."""

    def test_simulation_disabled_goes_directly_to_complete(self):
        h = Harness(simulation_phase_enabled=False)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived()
        self.assertEqual("WAITING_DELIVERY_SPEECH", h.orch.state)
        self.assertEqual([], h.actions("publish_simulation_navigation_goal"))
        h.speech_done()
        self.assertEqual("COMPLETE", h.orch.state)
        self.assertEqual([], h.actions("publish_simulation_navigation_goal"))
        self.assertEqual(
            "complete",
            h.actions("publish_status")[-1]["status"],
        )

    def test_matching_physical_arrival_publishes_simulation_goal(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived(status="arrived")
        self.assertEqual("WAITING_DELIVERY_SPEECH", h.orch.state)
        self.assertEqual([], h.actions("publish_simulation_navigation_goal"))
        speech = h.actions("publish_speech")[-1]
        self.assertEqual("delivery-speech-1", speech["speech_id"])
        self.assertEqual("已将苹果放入食品加工车间", speech["text"])
        h.speech_done()
        self.assertEqual("NAVIGATING_TO_SIM_WORKSHOP", h.orch.state)
        goal = h.actions("publish_simulation_navigation_goal")[-1]
        self.assertEqual(1, goal["protocol_version"])
        self.assertEqual(h.orch.task["task_id"], goal["task_id"])
        self.assertEqual("日用品加工车间", goal["target_workshop"])
        self.assertEqual("毛巾", goal["selected_item"])
        self.assertEqual(
            {"protocol_version", "task_id", "goal_id",
             "target_workshop", "selected_item"},
            set(goal),
        )
        h.simulation_arrived(goal_id=goal["goal_id"], status="arrived")
        self.assertEqual("WAITING_SIMULATION_SPEECH", h.orch.state)
        final_speech = h.actions("publish_speech")[-1]
        self.assertEqual("simulation-speech-1", final_speech["speech_id"])
        self.assertEqual(
            "仿真任务已完成，已将毛巾放入日用品加工车间",
            final_speech["text"],
        )
        h.speech_done()
        self.assertEqual("COMPLETE", h.orch.state)
        self.assertEqual(
            "complete",
            h.actions("publish_status")[-1]["status"],
        )
        # 终态重复结果只重发缓存状态，不重复运动。
        h.simulation_arrived(goal_id=goal["goal_id"], status="arrived")
        self.assertEqual(1, len(h.actions("publish_simulation_navigation_goal")))

    def test_physical_failure_never_publishes_simulation_goal(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived(status="failed")
        self.assertEqual("ERROR", h.orch.state)
        self.assertEqual([], h.actions("publish_simulation_navigation_goal"))

    def test_simulation_arrival_failure_enters_error_without_complete(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived()
        h.speech_done()
        h.simulation_arrived(status="failed")
        self.assertEqual("ERROR", h.orch.state)
        self.assertEqual(
            "navigation failed",
            h.actions("publish_status")[-1]["message"],
        )

    def test_first_arrival_never_publishes_complete(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived()
        self.assertEqual("WAITING_DELIVERY_SPEECH", h.orch.state)
        self.assertNotEqual(
            "complete",
            h.actions("publish_status")[-1]["status"],
        )

    def test_stale_simulation_results_never_advance(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived()
        h.speech_done()
        for label, message in (
            (
                "wrong_task",
                {"protocol_version": 1, "task_id": "task-2",
                 "goal_id": "simulation-1", "status": "arrived", "message": ""},
            ),
            (
                "wrong_goal",
                {"protocol_version": 1, "task_id": "task-1",
                 "goal_id": "stale-goal", "status": "arrived", "message": ""},
            ),
        ):
            with self.subTest(label=label):
                output_count = len(h.outputs)
                h.orch.on_simulation_arrived(message)
                self.assertEqual("NAVIGATING_TO_SIM_WORKSHOP", h.orch.state)
                self.assertEqual(output_count, len(h.outputs))

    def test_simulation_navigation_has_own_timeout(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived()
        h.speech_done()
        self.assertIsNotNone(h.orch.deadline)
        h.now[0] = h.orch.deadline + 0.01
        h.orch.tick()
        self.assertEqual("ERROR", h.orch.state)
        self.assertIn(
            "NAVIGATING_TO_SIM_WORKSHOP",
            h.actions("publish_status")[-1]["message"],
        )

    def test_cancel_works_in_simulation_navigation(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("NAVIGATING_TO_WORKSHOP")
        h.delivery_arrived()
        h.speech_done()
        h.orch.on_cancel(
            {"task_id": "task-1", "reason": "operator_cancel"}
        )
        self.assertEqual("CANCELLED", h.orch.state)
        self.assertEqual("IDLE", h.actions("publish_motion_mode")[-1])

    def test_parking_speech_failure_and_stale_identity_fail_closed(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("WAITING_DELIVERY_SPEECH")
        output_count = len(h.outputs)
        h.speech_done(speech_id="stale-speech")
        self.assertEqual("WAITING_DELIVERY_SPEECH", h.orch.state)
        self.assertEqual(output_count, len(h.outputs))
        h.speech_done(status="error")
        self.assertEqual("ERROR", h.orch.state)
        self.assertEqual([], h.actions("publish_simulation_navigation_goal"))

    def test_final_speech_timeout_prevents_complete(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("WAITING_SIMULATION_SPEECH")
        h.now[0] = h.orch.deadline + 0.01
        h.orch.tick()
        self.assertEqual("ERROR", h.orch.state)
        self.assertNotEqual("complete", h.actions("publish_status")[-1]["status"])

    def test_final_speech_failure_prevents_complete(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("WAITING_SIMULATION_SPEECH")
        h.speech_done(status="error")
        self.assertEqual("ERROR", h.orch.state)
        self.assertNotEqual("complete", h.actions("publish_status")[-1]["status"])

    def test_duplicate_arrivals_and_speech_done_do_not_repeat_actions(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("WAITING_DELIVERY_SPEECH")
        speech_count = len(h.actions("publish_speech"))
        h.delivery_arrived()
        self.assertEqual(speech_count, len(h.actions("publish_speech")))
        h.speech_done()
        goal_count = len(h.actions("publish_simulation_navigation_goal"))
        h.speech_done(speech_id="delivery-speech-1")
        self.assertEqual(goal_count, len(h.actions("publish_simulation_navigation_goal")))


class OrchestratorNavigationHandoffTests(unittest.TestCase):
    def test_ready_handoff_starts_bounded_stop_navigation(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("DELIVERY_HANDED_OFF")
        h.handoff_status()
        self.assertEqual("NAVIGATING_TO_WORKSHOP", h.orch.state)
        self.assertEqual("STOP_NAVIGATION", h.actions("publish_motion_mode")[-1])
        self.assertIsNotNone(h.orch.deadline)

    def test_failed_handoff_enters_error(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("DELIVERY_HANDED_OFF")
        h.handoff_status(status="failed")
        self.assertEqual("ERROR", h.orch.state)
        self.assertEqual("handoff failed", h.actions("publish_status")[-1]["message"])

    def test_arrival_before_ready_and_stale_handoff_are_ignored(self):
        h = Harness(simulation_phase_enabled=True)
        h.reach("DELIVERY_HANDED_OFF")
        output_count = len(h.outputs)
        h.delivery_arrived()
        h.handoff_status(goal_id="stale-delivery")
        self.assertEqual("DELIVERY_HANDED_OFF", h.orch.state)
        self.assertEqual(output_count, len(h.outputs))


class OrchestratorDeliveryHandoffTests(unittest.TestCase):
    """Lock the post-speech /task/delivery_navigation_goal contract."""

    def test_no_delivery_goal_is_published_before_matching_speech_done(self):
        h = Harness()
        h.reach("WAITING_SPEECH")
        self.assertEqual([], h.actions("publish_delivery_goal"))
        self.assertEqual("WAITING_SPEECH", h.orch.state)

    def test_failed_speech_never_publishes_delivery_goal(self):
        h = Harness()
        h.reach("WAITING_SPEECH")
        h.speech_done("error")
        self.assertEqual("ERROR", h.orch.state)
        self.assertEqual([], h.actions("publish_delivery_goal"))

    def test_matching_speech_done_publishes_exactly_one_full_goal(self):
        h = Harness()
        h.reach("WAITING_SPEECH")
        h.speech_done()
        goals = h.actions("publish_delivery_goal")
        self.assertEqual(1, len(goals))
        goal = goals[0]
        self.assertEqual(1, goal["protocol_version"])
        self.assertEqual("task-1", goal["task_id"])
        self.assertEqual("delivery-1", goal["goal_id"])
        self.assertEqual("食品加工车间", goal["target_workshop"])
        self.assertEqual("苹果", goal["selected_item"])
        self.assertEqual(
            {"protocol_version", "task_id", "goal_id",
             "target_workshop", "selected_item"},
            set(goal),
        )

    def test_handoff_state_is_idle_and_not_complete_without_fake_arrival(self):
        h = Harness()
        h.reach("DELIVERY_HANDED_OFF")
        self.assertEqual("DELIVERY_HANDED_OFF", h.orch.state)
        self.assertNotIn("COMPLETE", h.orch.state)
        self.assertEqual("IDLE", h.actions("publish_motion_mode")[-1])
        emitted_actions = [action for action, _payload in h.outputs]
        self.assertNotIn("publish_delivery_arrived", emitted_actions)
        self.assertNotEqual(
            "complete",
            h.actions("publish_status")[-1]["status"],
        )

    def test_repeated_speech_done_does_not_publish_goal_twice(self):
        h = Harness()
        h.reach("DELIVERY_HANDED_OFF")
        h.speech_done()
        self.assertEqual(1, len(h.actions("publish_delivery_goal")))
        self.assertEqual("DELIVERY_HANDED_OFF", h.orch.state)

    def test_wrong_or_stale_speech_id_does_not_publish_goal(self):
        h = Harness()
        h.reach("WAITING_SPEECH")
        h.speech_done(speech_id="stale-speech")
        self.assertEqual([], h.actions("publish_delivery_goal"))
        self.assertEqual("WAITING_SPEECH", h.orch.state)

    def test_other_task_speech_done_after_handoff_is_ignored(self):
        h = Harness()
        h.reach("DELIVERY_HANDED_OFF")
        goal_count = len(h.actions("publish_delivery_goal"))
        h.speech_done()
        h.speech_done()
        self.assertEqual(goal_count, len(h.actions("publish_delivery_goal")))
        self.assertEqual("DELIVERY_HANDED_OFF", h.orch.state)


if __name__ == "__main__":
    unittest.main()
