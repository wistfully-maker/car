import sys
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from line_follow_integration.protocol import (
    LINE_STATUSES,
    ProtocolError,
    build_arrival,
    build_line_status,
    parse_cancel,
    parse_identity_json,
    parse_navigation_goal,
)


class CancelTests(unittest.TestCase):
    def test_cancel_uses_existing_task_protocol_without_goal_id(self):
        parsed = parse_cancel(
            '{"protocol_version": 1, "task_id": "task-1", '
            '"reason": "operator_cancel"}'
        )
        self.assertEqual("task-1", parsed["task_id"])
        self.assertEqual("operator_cancel", parsed["reason"])
        self.assertNotIn("goal_id", parsed)

    def test_cancel_rejects_blank_or_padded_reason(self):
        for reason in ("", "  ", " operator_cancel "):
            raw = (
                '{"protocol_version": 1, "task_id": "task-1", '
                '"reason": %r}' % reason
            ).replace("'", '"')
            with self.subTest(reason=reason), self.assertRaises(ProtocolError):
                parse_cancel(raw)


class IdentityJsonTests(unittest.TestCase):
    def test_valid_identity_is_parsed_exactly(self):
        message = parse_identity_json(
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1"}'
        )
        self.assertEqual("task-1", message["task_id"])
        self.assertEqual("line-1", message["goal_id"])

    def test_non_object_json_is_rejected(self):
        for raw in ("[]", '"text"', "null", "42"):
            with self.subTest(raw=raw):
                with self.assertRaises(ProtocolError):
                    parse_identity_json(raw)

    def test_invalid_json_is_rejected(self):
        with self.assertRaises(ProtocolError):
            parse_identity_json("{not json")

    def test_unsupported_protocol_version_is_rejected(self):
        with self.assertRaises(ProtocolError):
            parse_identity_json(
                '{"protocol_version": 2, "task_id": "task-1", "goal_id": "line-1"}'
            )

    def test_blank_or_whitespace_padded_identities_are_rejected(self):
        for raw in (
            '{"protocol_version": 1, "task_id": "", "goal_id": "line-1"}',
            '{"protocol_version": 1, "task_id": " task-1", "goal_id": "line-1"}',
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": ""}',
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1 "}',
        ):
            with self.subTest(raw=raw):
                with self.assertRaises(ProtocolError):
                    parse_identity_json(raw)


class NavigationGoalTests(unittest.TestCase):
    def test_valid_pose_is_normalized_with_finite_values(self):
        message = parse_navigation_goal(
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1",'
            ' "pose": {"frame_id": "map", "x": 0.5, "y": -3.1,'
            ' "qz": -0.7, "qw": 0.7}}'
        )
        self.assertEqual("map", message["pose"]["frame_id"])
        self.assertAlmostEqual(0.5, message["pose"]["x"])
        self.assertAlmostEqual(-3.1, message["pose"]["y"])
        self.assertAlmostEqual(-0.7, message["pose"]["qz"])
        self.assertAlmostEqual(0.7, message["pose"]["qw"])

    def test_missing_or_non_object_pose_is_rejected(self):
        for raw in (
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1"}',
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1",'
            ' "pose": [1, 2]}',
        ):
            with self.subTest(raw=raw):
                with self.assertRaises(ProtocolError):
                    parse_navigation_goal(raw)

    def test_non_finite_pose_values_are_rejected(self):
        for field in ("x", "y", "qz", "qw"):
            for value in ("Infinity", "-Infinity", "NaN", '"text"', "true"):
                raw = (
                    '{"protocol_version": 1, "task_id": "task-1",'
                    ' "goal_id": "line-1", "pose": {"frame_id": "map",'
                    ' "x": 0.5, "y": -3.1, "qz": -0.7, "qw": 0.7}'
                )
                raw = raw.replace(
                    '"%s": 0.5' % field, '"%s": %s' % (field, value)
                ).replace(
                    '"%s": -3.1' % field, '"%s": %s' % (field, value)
                ).replace(
                    '"%s": -0.7' % field, '"%s": %s' % (field, value)
                ).replace(
                    '"%s": 0.7' % field, '"%s": %s' % (field, value)
                )
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ProtocolError):
                        parse_navigation_goal(raw)

    def test_zero_orientation_is_rejected(self):
        raw = (
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1",'
            ' "pose": {"frame_id": "map", "x": 0.5, "y": -3.1, "qz": 0.0,'
            ' "qw": 0.0}}'
        )
        with self.assertRaises(ProtocolError):
            parse_navigation_goal(raw)

    def test_blank_frame_id_is_rejected(self):
        raw = (
            '{"protocol_version": 1, "task_id": "task-1", "goal_id": "line-1",'
            ' "pose": {"frame_id": " ", "x": 0.5, "y": -3.1, "qz": -0.7,'
            ' "qw": 0.7}}'
        )
        with self.assertRaises(ProtocolError):
            parse_navigation_goal(raw)


class ArrivalBuilderTests(unittest.TestCase):
    def test_arrived_builds_without_message(self):
        payload = build_arrival("task-1", "line-1", True)
        self.assertEqual(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": "line-1",
                "status": "arrived",
                "message": "",
            },
            payload,
        )

    def test_failed_requires_message(self):
        with self.assertRaises(ProtocolError):
            build_arrival("task-1", "line-1", False)
        payload = build_arrival("task-1", "line-1", False, "  denied  ")
        self.assertEqual("denied", payload["message"])
        self.assertEqual("failed", payload["status"])

    def test_identity_and_succeeded_type_are_validated(self):
        for task_id in ("", " task-1"):
            with self.subTest(task_id=task_id):
                with self.assertRaises(ProtocolError):
                    build_arrival(task_id, "line-1", True)
        with self.assertRaises(ProtocolError):
            build_arrival("task-1", "", True)
        with self.assertRaises(ProtocolError):
            build_arrival("task-1", "line-1", "yes")


class LineStatusBuilderTests(unittest.TestCase):
    def test_waiting_signal_without_direction(self):
        payload = build_line_status("task-1", "line-1", "waiting_signal")
        self.assertEqual("waiting_signal", payload["status"])
        self.assertNotIn("direction", payload)
        self.assertNotIn("reason", payload)

    def test_direction_selected_and_following_require_direction(self):
        for status in ("direction_selected", "following"):
            with self.subTest(status=status):
                with self.assertRaises(ProtocolError):
                    build_line_status("task-1", "line-1", status)
                payload = build_line_status(
                    "task-1", "line-1", status, direction="left_turn"
                )
                self.assertEqual("left_turn", payload["direction"])

    def test_success_requires_direction(self):
        with self.assertRaises(ProtocolError):
            build_line_status("task-1", "line-1", "success")
        payload = build_line_status("task-1", "line-1", "success", direction="straight")
        self.assertEqual("success", payload["status"])

    def test_failure_requires_reason(self):
        with self.assertRaises(ProtocolError):
            build_line_status("task-1", "line-1", "failure")
        with self.assertRaises(ProtocolError):
            build_line_status("task-1", "line-1", "failure", reason="  ")
        payload = build_line_status(
            "task-1", "line-1", "failure", reason="  stale image  "
        )
        self.assertEqual("stale image", payload["reason"])

    def test_unknown_status_and_direction_are_rejected(self):
        with self.assertRaises(ProtocolError):
            build_line_status("task-1", "line-1", "unknown")
        with self.assertRaises(ProtocolError):
            build_line_status(
                "task-1", "line-1", "direction_selected", direction="u_turn"
            )

    def test_non_failure_status_cannot_carry_reason(self):
        with self.assertRaises(ProtocolError):
            build_line_status(
                "task-1", "line-1", "waiting_signal", reason="why"
            )

    def test_status_enum_matches_public_contract(self):
        self.assertEqual(
            frozenset(("waiting_signal", "direction_selected", "following",
                       "success", "failure")),
            LINE_STATUSES,
        )


if __name__ == "__main__":
    unittest.main()
