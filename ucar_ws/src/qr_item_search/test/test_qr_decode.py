import sys
import types
import unittest
from unittest.mock import Mock, patch

from qr_item_search.qr_decode import StableQrDecoder, pyzbar_backend


class PyzbarBackendTest(unittest.TestCase):
    def test_importing_qr_decode_does_not_load_pyzbar(self):
        self.assertNotIn("pyzbar.pyzbar", sys.modules)

    def test_decodes_only_qr_codes_as_trimmed_utf8(self):
        pyzbar_package = types.ModuleType("pyzbar")
        pyzbar_module = types.ModuleType("pyzbar.pyzbar")
        qr_code = object()
        pyzbar_module.ZBarSymbol = types.SimpleNamespace(QRCODE=qr_code)
        pyzbar_module.decode = Mock(
            return_value=[
                types.SimpleNamespace(
                    data=b"  https://example.test/\xe9\xa6\x99\xe8\x95\x89  "
                )
            ]
        )
        image = object()

        with patch.dict(
            sys.modules,
            {"pyzbar": pyzbar_package, "pyzbar.pyzbar": pyzbar_module},
        ):
            result = pyzbar_backend(image)

        self.assertEqual(["https://example.test/香蕉"], result)
        pyzbar_module.decode.assert_called_once_with(image, symbols=[qr_code])


class StableQrDecoderTest(unittest.TestCase):
    def test_confirms_same_value_on_two_frames(self):
        values = iter([["https://a.test"], ["https://a.test"]])
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=2,
        )

        self.assertIsNone(decoder.process(object()))
        self.assertEqual("https://a.test", decoder.process(object()))

    def test_different_value_resets_confirmation(self):
        values = iter(
            [["https://a.test"], ["https://b.test"], ["https://b.test"]]
        )
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=2,
        )

        self.assertIsNone(decoder.process(object()))
        self.assertIsNone(decoder.process(object()))
        self.assertEqual("https://b.test", decoder.process(object()))

    def test_reset_search_allows_same_qr_again(self):
        decoder = StableQrDecoder(
            backend=lambda image: ["https://a.test"],
            required_frames=1,
        )

        self.assertEqual("https://a.test", decoder.process(object()))
        self.assertIsNone(decoder.process(object()))
        decoder.reset_search()
        self.assertEqual("https://a.test", decoder.process(object()))

    def test_rejects_zero_required_frames(self):
        with self.assertRaises(ValueError):
            StableQrDecoder(required_frames=0)

    def test_no_result_resets_candidate(self):
        values = iter([["https://a.test"], [], ["https://a.test"], ["https://a.test"]])
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=2,
        )

        self.assertIsNone(decoder.process(object()))
        self.assertIsNone(decoder.process(object()))
        self.assertIsNone(decoder.process(object()))
        self.assertEqual("https://a.test", decoder.process(object()))

    def test_uses_first_non_empty_backend_value(self):
        decoder = StableQrDecoder(
            backend=lambda image: ["", "https://a.test", "https://b.test"],
            required_frames=1,
        )

        self.assertEqual("https://a.test", decoder.process(object()))

    def test_skips_seen_value_to_process_next_backend_value(self):
        values = iter(
            [
                ["https://a.test"],
                ["https://a.test", "https://b.test"],
            ]
        )
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=1,
        )

        self.assertEqual("https://a.test", decoder.process(object()))
        self.assertEqual("https://b.test", decoder.process(object()))

    def test_reset_wall_clears_candidate_but_keeps_seen_values(self):
        values = iter(
            [
                ["https://a.test"],
                ["https://a.test"],
                ["https://b.test"],
                ["https://b.test"],
                ["https://a.test"],
            ]
        )
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=2,
        )

        self.assertIsNone(decoder.process(object()))
        self.assertEqual("https://a.test", decoder.process(object()))
        self.assertIsNone(decoder.process(object()))
        decoder.reset_wall()
        self.assertIsNone(decoder.process(object()))
        self.assertIsNone(decoder.process(object()))


if __name__ == "__main__":
    unittest.main()
