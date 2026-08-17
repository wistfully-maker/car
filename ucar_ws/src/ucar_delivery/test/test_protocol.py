"""Phase A: lock the dual-stage delivery protocol contracts.

Covers strict parsing of both navigation goal types, strict building of both
arrival results, allowed workshop validation, identity preservation and
correlation rules. Pure logic: no ROS import.
"""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import protocol


def delivery_goal(**overrides):
    payload = {
        "protocol_version": 1,
        "task_id": "task-test-001",
        "goal_id": "delivery-test-001",
        "target_workshop": "食品加工车间",
        "selected_item": "苹果",
    }
    payload.update(overrides)
    return json.dumps(payload, ensure_ascii=False)


def simulation_goal(**overrides):
    payload = {
        "protocol_version": 1,
        "task_id": "task-test-001",
        "goal_id": "simulation-delivery-test-001",
        "target_workshop": "日用品加工车间",
        "selected_item": "毛巾",
    }
    payload.update(overrides)
    return json.dumps(payload, ensure_ascii=False)


class DeliveryGoalProtocolTests(unittest.TestCase):
    def test_parse_valid_delivery_goal(self):
        goal = protocol.parse_delivery_goal(delivery_goal())
        self.assertEqual(goal["task_id"], "task-test-001")
        self.assertEqual(goal["goal_id"], "delivery-test-001")
        self.assertEqual(goal["target_workshop"], "食品加工车间")
        self.assertEqual(goal["selected_item"], "苹果")

    def test_parse_valid_simulation_goal(self):
        goal = protocol.parse_simulation_goal(simulation_goal())
        self.assertEqual(goal["goal_id"], "simulation-delivery-test-001")
        self.assertEqual(goal["target_workshop"], "日用品加工车间")
        self.assertEqual(goal["selected_item"], "毛巾")

    def test_rejects_wrong_protocol_version(self):
        for parser in (protocol.parse_delivery_goal, protocol.parse_simulation_goal):
            with self.assertRaises(protocol.ProtocolError):
                parser(delivery_goal(protocol_version=2))
            with self.assertRaises(protocol.ProtocolError):
                parser(delivery_goal(protocol_version="1"))
            with self.assertRaises(protocol.ProtocolError):
                parser(delivery_goal(protocol_version=True))

    def test_rejects_missing_protocol_version(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(delivery_goal(protocol_version=None))

    def test_rejects_invalid_json(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal("not json {")
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal("42")
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal("[]")

    def test_allows_every_workshop(self):
        for workshop in protocol.ALLOWED_WORKSHOPS:
            goal = protocol.parse_delivery_goal(
                delivery_goal(target_workshop=workshop)
            )
            self.assertEqual(goal["target_workshop"], workshop)

    def test_rejects_unknown_workshop(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(
                delivery_goal(target_workshop="不存在的车间")
            )
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(delivery_goal(target_workshop=""))

    def test_rejects_empty_identity_fields(self):
        for field in ("task_id", "goal_id", "selected_item"):
            with self.assertRaises(protocol.ProtocolError):
                protocol.parse_delivery_goal(delivery_goal(**{field: ""}))
            with self.assertRaises(protocol.ProtocolError):
                protocol.parse_delivery_goal(delivery_goal(**{field: None}))

    def test_rejects_whitespace_padding(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(
                delivery_goal(target_workshop=" 食品加工车间 ")
            )
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(delivery_goal(task_id=" task-test-001 "))

    def test_rejects_non_text_fields(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(delivery_goal(task_id=42))
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_delivery_goal(delivery_goal(selected_item=["苹果"]))

    def test_phase_injected_by_parser_not_by_input(self):
        goal = protocol.parse_delivery_goal(
            delivery_goal(phase="simulation", **{})
        )
        self.assertEqual(goal["phase"], "physical")
        sim = protocol.parse_simulation_goal(
            simulation_goal(phase="physical", **{})
        )
        self.assertEqual(sim["phase"], "simulation")
        self.assertIn("protocol_version", goal)

    def test_simulation_goal_parser_rejects_delivery_goal_content(self):
        # The simulation parser is not required to be distinct, but must not
        # silently accept a goal that fails its own validation.
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_simulation_goal(
                simulation_goal(target_workshop="不存在车间")
            )


class ArrivalSerializationTests(unittest.TestCase):
    def test_build_delivery_arrival_success(self):
        arrival = protocol.build_arrival(
            "physical",
            "task-test-001",
            "delivery-test-001",
            "arrived",
            "",
        )
        self.assertEqual(arrival["protocol_version"], 1)
        self.assertEqual(arrival["task_id"], "task-test-001")
        self.assertEqual(arrival["goal_id"], "delivery-test-001")
        self.assertEqual(arrival["status"], "arrived")
        self.assertEqual(arrival["message"], "")

    def test_build_simulation_arrival_success(self):
        arrival = protocol.build_arrival(
            "simulation",
            "task-test-001",
            "simulation-delivery-test-001",
            "arrived",
            "",
        )
        self.assertEqual(arrival["goal_id"], "simulation-delivery-test-001")
        self.assertEqual(arrival["status"], "arrived")

    def test_build_arrival_failure_requires_message(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.build_arrival(
                "physical", "task-test-001", "delivery-test-001", "failed", ""
            )
        with self.assertRaises(protocol.ProtocolError):
            protocol.build_arrival(
                "physical", "task-test-001", "delivery-test-001", "failed", "  "
            )

    def test_build_arrival_failure_keeps_identifiers(self):
        arrival = protocol.build_arrival(
            "physical",
            "task-test-001",
            "delivery-test-001",
            "failed",
            "sign not found",
        )
        self.assertEqual(arrival["task_id"], "task-test-001")
        self.assertEqual(arrival["goal_id"], "delivery-test-001")
        self.assertEqual(arrival["message"], "sign not found")

    def test_build_arrival_success_message_normalized(self):
        arrival = protocol.build_arrival(
            "physical", "t", "g", "arrived", "  ok  "
        )
        self.assertEqual(arrival["message"], "ok")

    def test_build_arrival_rejects_unknown_status(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.build_arrival("physical", "t", "g", "done", "")

    def test_build_arrival_rejects_unknown_phase(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.build_arrival("magic", "t", "g", "arrived", "")

    def test_build_arrival_rejects_empty_identifiers(self):
        for task_id, goal_id in (("", "g"), ("t", ""), (None, "g")):
            with self.assertRaises(protocol.ProtocolError):
                protocol.build_arrival(
                    "physical", task_id, goal_id, "arrived", ""
                )

    def test_arrival_serializes_to_json_object(self):
        arrival = protocol.build_arrival(
            "physical", "t", "g", "arrived", ""
        )
        text = json.dumps(arrival, ensure_ascii=False)
        loaded = json.loads(text)
        self.assertEqual(loaded["status"], "arrived")


class PhaseCorrelationTests(unittest.TestCase):
    def test_phase_names_are_stable(self):
        self.assertEqual(protocol.PHASE_PHYSICAL, "physical")
        self.assertEqual(protocol.PHASE_SIMULATION, "simulation")

    def test_workshop_list_is_exact(self):
        self.assertEqual(
            protocol.ALLOWED_WORKSHOPS,
            ("食品加工车间", "日用品加工车间", "电子产品生产车间"),
        )


if __name__ == "__main__":
    unittest.main()
