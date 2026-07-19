import copy
import json
import math
import unittest

from qr_item_search.protocol import (
    EXPECTED_QR_COUNT,
    PROTOCOL_VERSION,
    ProtocolError,
    StartRequest,
    StopRequest,
    build_search_result,
    parse_start_request,
    parse_stop_request,
)


class StartRequestTest(unittest.TestCase):
    def test_parses_and_normalizes_valid_request(self):
        request = parse_start_request(json.dumps({
            "protocol_version": PROTOCOL_VERSION,
            "task_id": " task-1 ",
            "search_id": " search-1 ",
            "expected_count": EXPECTED_QR_COUNT,
        }))

        self.assertEqual(StartRequest("task-1", "search-1", 3), request)

    def test_rejects_non_json_and_non_object(self):
        for raw in ("not json", "[]", "null"):
            with self.subTest(raw=raw):
                with self.assertRaises(ProtocolError):
                    parse_start_request(raw)

    def test_rejects_missing_required_fields(self):
        raw = json.dumps({"protocol_version": 1, "task_id": "task"})
        with self.assertRaises(ProtocolError):
            parse_start_request(raw)

    def test_rejects_invalid_version_and_expected_count_types(self):
        base = {"protocol_version": 1, "task_id": "task", "search_id": "search", "expected_count": 3}
        for field, value in (("protocol_version", True), ("protocol_version", 1.0),
                             ("protocol_version", 2), ("expected_count", True),
                             ("expected_count", 3.0), ("expected_count", 2)):
            with self.subTest(field=field, value=value):
                raw = dict(base, **{field: value})
                with self.assertRaises(ProtocolError):
                    parse_start_request(json.dumps(raw))

    def test_rejects_empty_or_non_string_identifiers(self):
        base = {"protocol_version": 1, "task_id": "task", "search_id": "search", "expected_count": 3}
        for field, value in (("task_id", " \n "), ("task_id", 1),
                             ("search_id", ""), ("search_id", False)):
            with self.subTest(field=field, value=value):
                with self.assertRaises(ProtocolError):
                    parse_start_request(json.dumps(dict(base, **{field: value})))


class StopRequestTest(unittest.TestCase):
    def test_parses_and_normalizes_valid_request(self):
        request = parse_stop_request(json.dumps({
            "protocol_version": 1,
            "task_id": " task-1 ",
            "search_id": " search-1 ",
            "reason": " operator request ",
        }))

        self.assertEqual(StopRequest("task-1", "search-1", "operator request"), request)

    def test_rejects_missing_or_empty_text_fields(self):
        base = {"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"}
        for field, value in (("task_id", " "), ("search_id", ""), ("reason", "\t"), ("reason", 1)):
            with self.subTest(field=field, value=value):
                raw = dict(base, **{field: value})
                with self.assertRaises(ProtocolError):
                    parse_stop_request(json.dumps(raw))

    def test_rejects_non_integer_protocol_version(self):
        base = {"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"}
        for value in (True, 1.0, 2):
            with self.subTest(value=value):
                with self.assertRaises(ProtocolError):
                    parse_stop_request(json.dumps(dict(base, protocol_version=value)))


class SearchResultTest(unittest.TestCase):
    def _items(self):
        return [
            {"order": 1, "item_name": " apple ", "url": " https://one ", "detected_yaw": 1.0},
            {"order": 2, "item_name": "banana", "url": "https://two", "detected_yaw": -2},
            {"order": 3, "item_name": "carrot", "url": "https://three", "detected_yaw": 3},
        ]

    def test_builds_complete_result_in_order_and_normalizes_text(self):
        result = build_search_result(" task ", " search ", 1.5, "complete", self._items(), " done ")

        self.assertEqual({
            "protocol_version": 1, "task_id": "task", "search_id": "search", "stamp": 1.5,
            "status": "complete", "items": [
                {"order": 1, "item_name": "apple", "url": "https://one", "detected_yaw": 1.0},
                {"order": 2, "item_name": "banana", "url": "https://two", "detected_yaw": -2},
                {"order": 3, "item_name": "carrot", "url": "https://three", "detected_yaw": 3},
            ], "message": "done",
        }, result)

    def test_rejects_invalid_complete_items(self):
        cases = [
            self._items()[:2],
            [dict(item, url="https://one") for item in self._items()],
            [self._items()[0], dict(self._items()[1], order=3), self._items()[2]],
            [self._items()[0], "not a dict", self._items()[2]],
            [dict(self._items()[0], item_name=" "), self._items()[1], self._items()[2]],
            [dict(self._items()[0], url=""), self._items()[1], self._items()[2]],
        ]
        for items in cases:
            with self.subTest(items=items):
                with self.assertRaises(ProtocolError):
                    build_search_result("task", "search", 1, "complete", items, "")

    def test_rejects_non_finite_or_boolean_numeric_values(self):
        for stamp, items in ((True, self._items()), (math.inf, self._items()), (math.nan, self._items()),
                             (1, [dict(self._items()[0], detected_yaw=True)] + self._items()[1:]),
                             (1, [dict(self._items()[0], detected_yaw=math.nan)] + self._items()[1:]),
                             (1, [dict(self._items()[0], detected_yaw=-math.inf)] + self._items()[1:])):
            with self.subTest(stamp=stamp, items=items):
                with self.assertRaises(ProtocolError):
                    build_search_result("task", "search", stamp, "complete", items, "")

    def test_rejects_non_consecutive_or_non_integer_order(self):
        for order in (True, 1.0, 0, 2):
            items = [dict(self._items()[0], order=order)] + self._items()[1:]
            with self.subTest(order=order):
                with self.assertRaises(ProtocolError):
                    build_search_result("task", "search", 1, "complete", items, "")

    def test_rejects_invalid_status_and_result_identifiers(self):
        with self.assertRaises(ProtocolError):
            build_search_result("task", "search", 1, "unknown", [], "")
        for task_id, search_id in ((" ", "search"), ("task", ""), (1, "search")):
            with self.subTest(task_id=task_id, search_id=search_id):
                with self.assertRaises(ProtocolError):
                    build_search_result(task_id, search_id, 1, "searching", [], "")

    def test_terminal_status_requires_non_empty_message(self):
        for status in ("not_found", "error", "stopped"):
            with self.subTest(status=status):
                with self.assertRaises(ProtocolError):
                    build_search_result("task", "search", 1, status, [], " ")

    def test_searching_and_not_found_reject_three_items(self):
        for status in ("searching", "not_found"):
            with self.subTest(status=status):
                with self.assertRaises(ProtocolError):
                    build_search_result("task", "search", 1, status, self._items(), "reason")

    def test_every_status_rejects_more_than_expected_items(self):
        items = self._items() + [{
            "order": 4,
            "item_name": "date",
            "url": "https://four",
            "detected_yaw": 4,
        }]
        for status in ("searching", "complete", "not_found", "error", "stopped"):
            with self.subTest(status=status):
                with self.assertRaises(ProtocolError):
                    build_search_result("task", "search", 1, status, items, "reason")

    def test_non_complete_statuses_allow_partial_normalized_items(self):
        for status in ("searching", "not_found", "error", "stopped"):
            for count in (1, 2):
                with self.subTest(status=status, count=count):
                    result = build_search_result(
                        "task", "search", 1, status, self._items()[:count], " reason "
                    )
                    self.assertEqual(count, len(result["items"]))
                    self.assertEqual("apple", result["items"][0]["item_name"])
                    self.assertEqual("https://one", result["items"][0]["url"])
                    self.assertEqual("reason", result["message"])

    def test_searching_allows_empty_message_but_rejects_non_string(self):
        result = build_search_result("task", "search", 1, "searching", [], "")
        self.assertEqual("", result["message"])
        with self.assertRaises(ProtocolError):
            build_search_result("task", "search", 1, "searching", [], None)

    def test_does_not_mutate_items_input(self):
        items = self._items()
        original = copy.deepcopy(items)
        build_search_result("task", "search", 1, "complete", items, "")
        self.assertEqual(original, items)


if __name__ == "__main__":
    unittest.main()
