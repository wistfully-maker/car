"""Vehicle YOLO + OCR adapter contract tests (fake-ROS).

These tests lock both the original narrow test contract and the deployed
vehicle contract (yolo_biao.infer + ocr.ocr_infer):

- both modules must be importable at node startup; a missing OCR module is a
  fail-fast startup error (SystemExit), never a silently degraded mission;
- the YOLO candidate must carry timestamp/target/confidence/bbox and the
  separate OCR detector must supply matching text
  (the staging estimate depends on bbox-derived bearing);
- a valid vehicle candidate confirmed over consecutive frames must publish
  /task/delivery_sign_found with bbox/confidence/ocr_text and the derived
  bearing fields.
"""

import json
import logging
import sys
import types
import unittest

from test_nodes import (
    FakeString,
    RosHarness,
    mission_params,
)

VEHICLE_PARAMS = {
    "backend": "vehicle",
    "vehicle_backend_module": "yolo_biao.infer",
    "vehicle_ocr_module": "ocr.infer",
    "min_confidence": 0.6,
    "confirm_frames": 3,
    "ocr_confirm_frames": 1,
    "max_image_age": 1.0,
    "target_aliases": {},
}


def vehicle_candidate():
    return {
        "timestamp": 100.0,
        "target": "食品加工车间",
        "confidence": 0.95,
        "bbox": [100, 80, 140, 120],
        "detection_yaw": 0.1,
    }


class FakeYoloDetector:
    def __init__(self, candidate=None):
        self._candidate = candidate
        self.calls = 0

    def detect(self, image, timestamp):
        self.calls += 1
        if self._candidate is None:
            return None
        return dict(self._candidate)


class FakeOcrDetector:
    def __init__(self):
        self.calls = []
        self.result = {"text": "食品加工车间", "confidence": 0.9}

    def detect(self, image, bbox, timestamp):
        self.calls.append((image, list(bbox), timestamp))
        return self.result


class FakePredictYoloDetector:
    def __init__(self):
        self.calls = []
        self.result = [[100.0, 80.0, 140.0, 120.0, 0.95]]

    def predict(self, image):
        self.calls.append(image)
        return self.result


class FakeRapidOcrInfer:
    def __init__(self):
        self.calls = []
        self.result = ("食品加工车间", 0.9)

    def predict(self, image):
        self.calls.append(image)
        return self.result


def install_vehicle_modules(yolo_ok=True, ocr_ok=True):
    """Inject fake yolo_biao.infer / ocr.infer modules into sys.modules."""
    if "yolo_biao" in sys.modules:
        del sys.modules["yolo_biao"]
    if "yolo_biao.infer" in sys.modules:
        del sys.modules["yolo_biao.infer"]
    if "ocr" in sys.modules:
        del sys.modules["ocr"]
    if "ocr.infer" in sys.modules:
        del sys.modules["ocr.infer"]
    if "ocr.ocr_infer" in sys.modules:
        del sys.modules["ocr.ocr_infer"]
    if yolo_ok:
        yolo = types.ModuleType("yolo_biao")
        infer = types.ModuleType("yolo_biao.infer")
        infer.YoloDetector = FakeYoloDetector
        yolo.infer = infer
        sys.modules["yolo_biao"] = yolo
        sys.modules["yolo_biao.infer"] = infer
    if ocr_ok:
        ocr = types.ModuleType("ocr")
        ocr_infer = types.ModuleType("ocr.infer")
        ocr_infer.OcrDetector = FakeOcrDetector
        ocr.infer = ocr_infer
        sys.modules["ocr"] = ocr
        sys.modules["ocr.infer"] = ocr_infer


def install_vehicle_predict_modules():
    install_vehicle_modules()
    sys.modules["yolo_biao.infer"].YoloDetector = FakePredictYoloDetector
    ocr_predict = types.ModuleType("ocr.ocr_infer")
    ocr_predict.RapidOcrInfer = FakeRapidOcrInfer
    sys.modules["ocr.ocr_infer"] = ocr_predict


class FakeImage:
    def __init__(self, width=64, height=48):
        import numpy as np
        self.width = width
        self.height = height
        self.encoding = "mono8"
        self.data = bytes(np.zeros((height, width), dtype=np.uint8).tobytes())
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: 100.0)
        )


class VehicleBackendLoadTests(unittest.TestCase):
    def setUp(self):
        self.harness = RosHarness({})
        install_vehicle_modules()
        self.addCleanup(install_vehicle_modules)

    def load_backend_module(self):
        import importlib.util
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            "ucar_delivery_sign_adapter",
            root / "scripts" / "sign_adapter_node.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_vehicle_backend_composes_yolo_and_ocr_detectors(self):
        module = self.load_backend_module()
        backend = module._load_vehicle_backend(
            "yolo_biao.infer", "ocr.infer"
        )
        self.assertIsInstance(backend._yolo, FakeYoloDetector)
        self.assertIsInstance(backend._ocr, FakeOcrDetector)

        backend._yolo._candidate = vehicle_candidate()
        image = object()
        candidate = backend.detect(image, 100.0)
        self.assertEqual("食品加工车间", candidate["ocr_text"])
        self.assertEqual(0.9, candidate["ocr_confidence"])
        self.assertEqual([(image, [100, 80, 140, 120], 100.0)],
                         backend._ocr.calls)

    def test_missing_ocr_module_fails_fast(self):
        install_vehicle_modules(ocr_ok=False)
        module = self.load_backend_module()
        with self.assertRaises(RuntimeError) as ctx:
            module._load_vehicle_backend("yolo_biao.infer", "ocr.infer")
        self.assertIn("ocr.infer", str(ctx.exception))

    def test_missing_yolo_detector_class_fails_fast(self):
        install_vehicle_modules(yolo_ok=False)
        module = self.load_backend_module()
        with self.assertRaises(RuntimeError):
            module._load_vehicle_backend("yolo_biao.infer", "ocr.infer")

    def test_missing_ocr_detector_class_fails_fast(self):
        install_vehicle_modules()
        del sys.modules["ocr.infer"].OcrDetector
        module = self.load_backend_module()
        with self.assertRaises(RuntimeError):
            module._load_vehicle_backend("yolo_biao.infer", "ocr.infer")

    def test_wrong_ocr_detect_signature_fails_fast(self):
        install_vehicle_modules()

        class WrongOcrDetector:
            def detect(self, image):
                return "食品加工车间"

        sys.modules["ocr.infer"].OcrDetector = WrongOcrDetector
        module = self.load_backend_module()
        with self.assertRaises(RuntimeError):
            module._load_vehicle_backend("yolo_biao.infer", "ocr.infer")

    def test_vehicle_predict_contract_matches_deployed_modules(self):
        install_vehicle_predict_modules()
        module = self.load_backend_module()
        backend = module._load_vehicle_backend(
            "yolo_biao.infer", "ocr.ocr_infer"
        )
        import numpy as np
        image = np.zeros((240, 320, 3), dtype=np.uint8)

        candidate = backend.detect(image, 100.0)

        self.assertEqual([100.0, 80.0, 140.0, 120.0], candidate["bbox"])
        self.assertEqual(0.95, candidate["confidence"])
        self.assertEqual("食品加工车间", candidate["target"])
        self.assertEqual("食品加工车间", candidate["ocr_text"])
        self.assertEqual(0.9, candidate["ocr_confidence"])
        self.assertEqual((40, 40, 3), backend._ocr.calls[0].shape)

    def test_vehicle_loader_repairs_rknn_logging_level_pollution(self):
        install_vehicle_predict_modules()
        original_info_name = logging.getLevelName(logging.INFO)

        class LoggingPollutingYolo(FakePredictYoloDetector):
            def __init__(self):
                super().__init__()
                logging.addLevelName(logging.INFO, "I")

        sys.modules["yolo_biao.infer"].YoloDetector = LoggingPollutingYolo
        module = self.load_backend_module()
        try:
            module._load_vehicle_backend(
                "yolo_biao.infer", "ocr.ocr_infer"
            )
            self.assertEqual("INFO", logging.getLevelName(logging.INFO))
        finally:
            logging.addLevelName(logging.INFO, original_info_name)


class VehicleNodeFailFastTests(unittest.TestCase):
    def setUp(self):
        install_vehicle_modules(ocr_ok=True)
        self.addCleanup(install_vehicle_modules)

    def params(self, **overrides):
        params = mission_params()
        sign = dict(VEHICLE_PARAMS)
        sign.update(overrides)
        params["~sign_detector"] = sign
        return params

    def test_missing_ocr_module_node_exits_fast(self):
        install_vehicle_modules(ocr_ok=False)
        self.harness = RosHarness(self.params())
        with self.assertRaises(SystemExit):
            self.harness.load_script(
                "scripts/sign_adapter_node.py", "SignAdapterNode"
            )
        # fail-fast：节点退出，无任何订阅/发布（不可能进入可运动状态）

    def test_vehicle_node_constructs_when_contract_ok(self):
        self.harness = RosHarness(self.params())
        node, _module = self.harness.load_script(
            "scripts/sign_adapter_node.py", "SignAdapterNode"
        )
        self.assertIsNotNone(node._detector)
        self.assertIn("/usb_cam/image_raw", self.harness.subscribers)


class VehicleCandidateContractTests(unittest.TestCase):
    def setUp(self):
        install_vehicle_modules()
        self.addCleanup(install_vehicle_modules)

    def test_confirmed_vehicle_candidate_publishes_full_contract(self):
        params = dict(mission_params())
        params["~sign_detector"] = dict(VEHICLE_PARAMS)
        self.harness = RosHarness(params)
        node, _module = self.harness.load_script(
            "scripts/sign_adapter_node.py", "SignAdapterNode"
        )
        # vehicle 后端输出满足合同的候选
        node._detector._backend._yolo._candidate = vehicle_candidate()
        # 启动标牌搜索
        self.harness.subscribers["/task/delivery_sign_start"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "target_workshop": "食品加工车间",
            }, ensure_ascii=False)
        ))
        # 连续 3 帧确认
        for _ in range(3):
            self.harness.subscribers["/usb_cam/image_raw"](FakeImage())
        found = self.harness.messages.get("/task/delivery_sign_found", [])
        self.assertEqual(1, len(found))
        detection = json.loads(found[0].data)["detection"]
        # 合同字段：bbox / confidence / ocr_text 必须存在
        for key in ("bbox", "confidence", "ocr_text", "ocr_confirmed",
                    "target_name", "timestamp"):
            self.assertIn(key, detection)
        self.assertEqual(4, len(detection["bbox"]))
        self.assertIsInstance(detection["confidence"], float)
        self.assertTrue(detection["ocr_confirmed"])
        # staging 依赖的 bearing 必须由 bbox 推导
        self.assertIn("bearing_rad", detection)
        self.assertIn("half_width_rad", detection)

    def test_malformed_ocr_result_never_confirms(self):
        params = dict(mission_params())
        params["~sign_detector"] = dict(VEHICLE_PARAMS)
        self.harness = RosHarness(params)
        node, _module = self.harness.load_script(
            "scripts/sign_adapter_node.py", "SignAdapterNode"
        )
        node._detector._backend._yolo._candidate = vehicle_candidate()
        node._detector._backend._ocr.result = {"confidence": 0.9}
        self.harness.subscribers["/task/delivery_sign_start"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "target_workshop": "食品加工车间",
            }, ensure_ascii=False)
        ))
        for _ in range(3):
            self.harness.subscribers["/usb_cam/image_raw"](FakeImage())
        self.assertEqual([], self.harness.messages["/task/delivery_sign_found"])

    def test_vehicle_candidate_missing_fields_never_confirms(self):
        self.harness = RosHarness(mission_params())
        params = dict(mission_params())
        sign = dict(VEHICLE_PARAMS)
        params["~sign_detector"] = sign
        self.harness = RosHarness(params)
        node, _module = self.harness.load_script(
            "scripts/sign_adapter_node.py", "SignAdapterNode"
        )
        self.harness.subscribers["/task/delivery_sign_start"](FakeString(
            json.dumps({
                "protocol_version": 1, "phase": "physical",
                "task_id": "task-1", "goal_id": "delivery-1",
                "target_workshop": "食品加工车间",
            }, ensure_ascii=False)
        ))
        # 后端输出缺 bbox 的候选：永不确认 → 不发布 sign_found
        broken = vehicle_candidate()
        broken.pop("bbox")
        node._detector._backend._yolo._candidate = broken
        for _ in range(5):
            self.harness.subscribers["/usb_cam/image_raw"](FakeImage())
        found = self.harness.messages.get("/task/delivery_sign_found", [])
        self.assertEqual(0, len(found))
        self.assertIsNotNone(node._detector.last_error)


if __name__ == "__main__":
    unittest.main()
