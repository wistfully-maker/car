#!/usr/bin/env python3

import json
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from ucar_waypoint_nav.protocol import (  # noqa: E402
    make_pickup_result,
    parse_pickup_goal,
)


class ProtocolTests(unittest.TestCase):
    def test_preserves_task_identity_in_arrival_result(self):
        identity = parse_pickup_goal(
            '{"protocol_version":1,"task_id":"task-001",'
            '"goal_id":"pickup-001"}'
        )
        result = json.loads(
            make_pickup_result(identity, "arrived", "")
        )
        self.assertEqual(result["protocol_version"], 1)
        self.assertEqual(result["task_id"], "task-001")
        self.assertEqual(result["goal_id"], "pickup-001")
        self.assertEqual(result["status"], "arrived")

    def test_rejects_missing_task_id(self):
        with self.assertRaisesRegex(ValueError, "task_id"):
            parse_pickup_goal(
                '{"protocol_version":1,"goal_id":"pickup-001"}'
            )

    def test_rejects_wrong_protocol_version(self):
        with self.assertRaisesRegex(ValueError, "protocol_version"):
            parse_pickup_goal(
                '{"protocol_version":2,"task_id":"task-001",'
                '"goal_id":"pickup-001"}'
            )

    def test_failure_requires_a_message(self):
        identity = parse_pickup_goal(
            '{"protocol_version":1,"task_id":"task-001",'
            '"goal_id":"pickup-001"}'
        )
        with self.assertRaisesRegex(ValueError, "message"):
            make_pickup_result(identity, "failed", "")


if __name__ == "__main__":
    unittest.main()
