import unittest
from unittest.mock import Mock

import requests

from qr_item_search.qr_payload import InvalidPayload, InvalidQrUrl, ItemResolver


class ItemResolverTest(unittest.TestCase):
    def setUp(self):
        self.session = Mock()
        self.resolver = ItemResolver(session=self.session)

    def test_rejects_non_http_url(self):
        with self.assertRaises(InvalidQrUrl):
            self.resolver.resolve("not-a-url")

        self.session.get.assert_not_called()

    def test_returns_trimmed_result_from_successful_payload(self):
        response = Mock()
        response.json.return_value = {"code": 200, "result": " 香蕉 "}
        self.session.get.return_value = response

        result = self.resolver.resolve(" \nhttps://example.com/item\t ")

        self.assertEqual("香蕉", result)
        self.session.get.assert_called_once_with(
            "https://example.com/item", timeout=(1.0, 2.0)
        )

    def test_rejects_non_success_business_code(self):
        response = Mock()
        response.json.return_value = {"code": 400, "result": "香蕉"}
        self.session.get.return_value = response

        with self.assertRaises(InvalidPayload):
            self.resolver.resolve("https://example.com/item")

        self.session.get.assert_called_once()

    def test_retries_once_after_network_error(self):
        response = Mock()
        response.json.return_value = {"code": 200, "result": "苹果"}
        self.session.get.side_effect = [requests.RequestException("temporary"), response]
        resolver = ItemResolver(session=self.session, retries=1)

        self.assertEqual("苹果", resolver.resolve("https://example.com/item"))
        self.assertEqual(2, self.session.get.call_count)

    def test_rejects_missing_or_empty_result(self):
        for payload in ({"code": 200}, {"code": 200, "result": "  "}):
            with self.subTest(payload=payload):
                response = Mock()
                response.json.return_value = payload
                self.session.get.return_value = response

                with self.assertRaises(InvalidPayload):
                    self.resolver.resolve("https://example.com/item")

    def test_rejects_json_payload_that_is_not_an_object(self):
        response = Mock()
        response.json.return_value = []
        self.session.get.return_value = response

        with self.assertRaises(InvalidPayload):
            self.resolver.resolve("https://example.com/item")

    def test_rejects_negative_retry_count(self):
        with self.assertRaises(ValueError):
            ItemResolver(retries=-1)


if __name__ == "__main__":
    unittest.main()
