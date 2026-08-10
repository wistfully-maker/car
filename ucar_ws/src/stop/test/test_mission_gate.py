import sys
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from stop_integration.mission_gate import MissionGate


def full_mission(**overrides):
    mission = {
        "protocol_version": 1,
        "task_id": "task-1",
        "physical_goal_id": "delivery-1",
        "simulation_goal_id": "simulation-1",
        "physical": {"target_workshop": "食品加工车间", "selected_item": "苹果"},
        "simulation": {"target_workshop": "日用品加工车间", "selected_item": "毛巾"},
    }
    mission.update(overrides)
    return mission


class MissionGateAcceptanceTests(unittest.TestCase):
    def test_accepts_valid_correlated_mission(self):
        gate = MissionGate()
        accepted = gate.accept(full_mission())
        self.assertTrue(accepted.start)
        self.assertIsNone(accepted.error)
        self.assertEqual("task-1", accepted.mission["task_id"])
        self.assertEqual("delivery-1", accepted.mission["physical_goal_id"])
        self.assertEqual("苹果", accepted.mission["physical"]["selected_item"])

    def test_accepting_alone_never_starts_motion(self):
        gate = MissionGate()
        accepted = gate.accept(full_mission())
        self.assertTrue(accepted.start)
        # 门控只是校验与记录；是否运动由上层 activate() 决定。
        self.assertFalse(hasattr(gate, "activate"))

    def test_duplicate_active_identity_does_not_restart(self):
        gate = MissionGate()
        gate.accept(full_mission())
        second = gate.accept(full_mission())
        self.assertFalse(second.start)
        self.assertIsNone(second.error)
        self.assertEqual("task-1", second.mission["task_id"])

    def test_cross_task_update_while_active_is_rejected(self):
        gate = MissionGate()
        gate.accept(full_mission())
        second = gate.accept(full_mission(task_id="task-2"))
        self.assertFalse(second.start)
        self.assertIsNotNone(second.error)
        self.assertIn("task-1", second.error)

    def test_duplicate_terminal_identity_returns_cached_result(self):
        gate = MissionGate()
        gate.accept(full_mission())
        gate.record_result("task-1", "done", "")
        cached = gate.accept(full_mission())
        self.assertFalse(cached.start)
        self.assertIsNotNone(cached.terminal)
        self.assertEqual("done", cached.terminal["status"])
        self.assertEqual("task-1", cached.terminal["task_id"])

    def test_terminal_mission_accepts_a_new_task(self):
        gate = MissionGate()
        gate.accept(full_mission())
        gate.record_result("task-1", "done", "")
        accepted = gate.accept(full_mission(task_id="task-2",
                                            physical_goal_id="delivery-2",
                                            simulation_goal_id="simulation-2"))
        self.assertTrue(accepted.start)
        self.assertEqual("task-2", accepted.mission["task_id"])


class MissionGateValidationTests(unittest.TestCase):
    def test_protocol_mismatch_is_rejected(self):
        for version in (2, True, "1", None):
            with self.subTest(version=version):
                gate = MissionGate()
                accepted = gate.accept(full_mission(protocol_version=version))
                self.assertFalse(accepted.start)
                self.assertIsNotNone(accepted.error)

    def test_empty_ids_are_rejected(self):
        for field in ("task_id", "physical_goal_id", "simulation_goal_id"):
            with self.subTest(field=field):
                gate = MissionGate()
                accepted = gate.accept(full_mission(**{field: "  "}))
                self.assertFalse(accepted.start)
                self.assertIsNotNone(accepted.error)

    def test_malformed_physical_targets_are_rejected(self):
        for field in ("target_workshop", "selected_item"):
            for value in ("", "  ", 3, None, ["苹果"]):
                with self.subTest(field=field, value=value):
                    gate = MissionGate()
                    mission = full_mission()
                    mission["physical"][field] = value
                    accepted = gate.accept(mission)
                    self.assertFalse(accepted.start)
                    self.assertIsNotNone(accepted.error)

    def test_malformed_simulation_targets_are_rejected(self):
        for field in ("target_workshop", "selected_item"):
            with self.subTest(field=field):
                gate = MissionGate()
                mission = full_mission()
                mission["simulation"][field] = " "
                accepted = gate.accept(mission)
                self.assertFalse(accepted.start)
                self.assertIsNotNone(accepted.error)

    def test_non_object_input_is_rejected(self):
        for value in (None, "text", ["list"], 3):
            with self.subTest(value=value):
                gate = MissionGate()
                accepted = gate.accept(value)
                self.assertFalse(accepted.start)
                self.assertIsNotNone(accepted.error)

    def test_unknown_mission_keys_do_not_break_validation(self):
        gate = MissionGate()
        mission = full_mission(extra_field="forward-compatible")
        accepted = gate.accept(mission)
        self.assertTrue(accepted.start)
        self.assertEqual("task-1", accepted.mission["task_id"])

    def test_physical_only_mission_is_accepted(self):
        gate = MissionGate()
        mission = full_mission()
        del mission["simulation"]
        accepted = gate.accept(mission)
        self.assertTrue(accepted.start)
        self.assertNotIn("simulation", accepted.mission)


class MissionGateSimulationUpdateTests(unittest.TestCase):
    def test_update_simulation_fills_active_mission(self):
        gate = MissionGate()
        mission = full_mission()
        del mission["simulation"]
        gate.accept(mission)
        updated = gate.update_simulation(
            "task-1", "simulation-1",
            {"target_workshop": "日用品加工车间", "selected_item": "毛巾"},
        )
        self.assertEqual("simulation-1", updated["simulation_goal_id"])
        self.assertEqual("毛巾", updated["simulation"]["selected_item"])

    def test_update_simulation_for_wrong_task_is_rejected(self):
        gate = MissionGate()
        mission = full_mission()
        del mission["simulation"]
        gate.accept(mission)
        with self.assertRaises(ValueError):
            gate.update_simulation(
                "task-2", "simulation-1",
                {"target_workshop": "日用品加工车间", "selected_item": "毛巾"},
            )

    def test_update_simulation_without_active_mission_is_rejected(self):
        gate = MissionGate()
        with self.assertRaises(ValueError):
            gate.update_simulation(
                "task-1", "simulation-1",
                {"target_workshop": "日用品加工车间", "selected_item": "毛巾"},
            )

    def test_update_simulation_rejects_empty_targets(self):
        gate = MissionGate()
        mission = full_mission()
        del mission["simulation"]
        gate.accept(mission)
        with self.assertRaises(ValueError):
            gate.update_simulation(
                "task-1", "simulation-1",
                {"target_workshop": " ", "selected_item": "毛巾"},
            )

    def test_full_mission_after_update_completes_terminal(self):
        gate = MissionGate()
        mission = full_mission()
        del mission["simulation"]
        gate.accept(mission)
        gate.update_simulation(
            "task-1", "simulation-1",
            {"target_workshop": "日用品加工车间", "selected_item": "毛巾"},
        )
        gate.record_result("task-1", "done", "")
        cached = gate.accept(full_mission())
        self.assertFalse(cached.start)
        self.assertEqual("done", cached.terminal["status"])


class MissionGateResultTests(unittest.TestCase):
    def test_record_result_clears_active_mission(self):
        gate = MissionGate()
        gate.accept(full_mission())
        gate.record_result("task-1", "failed", "not_found")
        accepted = gate.accept(full_mission())
        self.assertFalse(accepted.start)
        self.assertEqual("failed", accepted.terminal["status"])
        self.assertEqual("not_found", accepted.terminal["message"])

    def test_repeated_terminal_query_returns_same_cache(self):
        gate = MissionGate()
        gate.accept(full_mission())
        gate.record_result("task-1", "done", "")
        first = gate.accept(full_mission())
        second = gate.accept(full_mission())
        self.assertEqual(first.terminal, second.terminal)


if __name__ == "__main__":
    unittest.main()
