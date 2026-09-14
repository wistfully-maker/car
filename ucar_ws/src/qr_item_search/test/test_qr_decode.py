import sys
import types
import unittest
from unittest.mock import Mock, patch

from qr_item_search.qr_decode import UniqueQrDecoder, pyzbar_backend, pyzbar_backend_with_rect


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

    def test_decodes_urls_with_optional_rects(self):
        pyzbar_package = types.ModuleType("pyzbar")
        pyzbar_module = types.ModuleType("pyzbar.pyzbar")
        qr_code = object()
        rect = types.SimpleNamespace(x=1, y=2, width=30, height=30)
        pyzbar_module.ZBarSymbol = types.SimpleNamespace(QRCODE=qr_code)
        pyzbar_module.decode = Mock(
            return_value=[
                types.SimpleNamespace(data=b"https://a.test", rect=rect),
                types.SimpleNamespace(data=b"https://b.test", rect=rect),
            ]
        )
        image = object()

        with patch.dict(
            sys.modules,
            {"pyzbar": pyzbar_package, "pyzbar.pyzbar": pyzbar_module},
        ):
            result = pyzbar_backend_with_rect(image)

        self.assertEqual(["https://a.test", "https://b.test"], [url for url, _ in result])
        self.assertEqual(rect, result[0][1])


class UniqueQrDecoderTest(unittest.TestCase):
    def test_returns_multiple_trimmed_values_in_backend_order(self):
        decoder = UniqueQrDecoder(backend=lambda image: [" https://a.test ", "https://b.test"])
        self.assertEqual(["https://a.test", "https://b.test"], decoder.process(object()))

    def test_deduplicates_within_and_across_frames(self):
        values = iter([["https://a.test", "https://a.test", "https://b.test"],
                       ["https://a.test", "https://c.test"]])
        decoder = UniqueQrDecoder(backend=lambda image: next(values))
        self.assertEqual(["https://a.test", "https://b.test"], decoder.process(object()))
        self.assertEqual(["https://c.test"], decoder.process(object()))

    def test_reset_search_allows_values_again(self):
        decoder = UniqueQrDecoder(backend=lambda image: ["https://a.test"])
        self.assertEqual(["https://a.test"], decoder.process(object()))
        self.assertEqual([], decoder.process(object()))
        decoder.reset_search()
        self.assertEqual(["https://a.test"], decoder.process(object()))

    def test_ignores_empty_values(self):
        decoder = UniqueQrDecoder(backend=lambda image: ["", "  "])
        self.assertEqual([], decoder.process(object()))

    def test_non_string_value_does_not_partially_update_seen_values(self):
        decoder = UniqueQrDecoder(backend=lambda image: ["https://a.test", 42])
        with self.assertRaises(TypeError):
            decoder.process(object())
        decoder._backend = lambda image: ["https://a.test"]
        self.assertEqual(["https://a.test"], decoder.process(object()))

    def test_backend_runtime_error_propagates_without_updating_seen_values(self):
        def failing_backend(image):
            raise RuntimeError("camera unavailable")

        decoder = UniqueQrDecoder(backend=failing_backend)
        with self.assertRaisesRegex(RuntimeError, "camera unavailable"):
            decoder.process(object())
        decoder._backend = lambda image: ["https://a.test"]
        self.assertEqual(["https://a.test"], decoder.process(object()))

    def test_rejects_non_string_backend_values(self):
        decoder = UniqueQrDecoder(backend=lambda image: [42])
        with self.assertRaises(TypeError):
            decoder.process(object())


if __name__ == "__main__":
    unittest.main()
