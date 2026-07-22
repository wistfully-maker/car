import json
import unittest

from qr_item_search.controller_logic import SearchController
from qr_item_search.qr_decode import UniqueQrDecoder
from qr_item_search.scanner_logic import ScannerLogic


class Outputs:
    def __init__(self):
        self.speeds, self.states, self.controls, self.results = [], [], [], []
    def publish_speed(self, value): self.speeds.append(value)
    def publish_state(self, value): self.states.append(value)
    def publish_scanner_control(self, value): self.controls.append(value)
    def publish_result(self, value): self.results.append(value)


class Resolver:
    def resolve(self, url):
        return {"https://1": "one", "https://2": "two", "https://3": "three"}[url]


class IntegrationContractTest(unittest.TestCase):
    def setUp(self):
        self.outputs = Outputs()
        self.controller = SearchController(self.outputs)
        self.controller.update_yaw(0.0, 0.0)
        self.controller.start(json.dumps({"protocol_version": 1, "task_id": "task",
                                          "search_id": "search", "expected_count": 3}), 0.0)
        self.now = 0.01
        self.events = []
        self.scanner_session = "session-a"

    def publish(self, value):
        value = dict(value)
        value["scanner_session"] = self.scanner_session
        self.events.append(value)
        self.now += 0.01
        self.controller.handle_scanner_event(json.dumps(value), self.now)

    def logic(self, values):
        return ScannerLogic(
            decoder=UniqueQrDecoder(backend=lambda image: values), resolver=Resolver(),
            event_publisher=self.publish,
            quality_function=lambda image: {"brightness": 50.0, "overexposed": 0.0, "sharpness": 50.0},
            variant_function=lambda image, enhanced: [image], worker_count=1,
        )

    def test_real_events_complete_controller_in_order(self):
        logic = self.logic(["https://1", "https://2", "https://3"])
        logic.reset_search("task", "search"); logic.set_control(True, False, 0.5)
        logic.submit_frame(object()); logic.process_latest_frame()
        for _ in range(3): logic.work_once(block=False)
        self.assertEqual("COMPLETE", self.controller.state)
        self.assertEqual(["one", "two", "three"],
                         [item["item_name"] for item in self.outputs.results[-1]["items"]])
        self.assertTrue(all(event["protocol_version"] == 1 for event in self.events))

    def test_invalid_url_event_does_not_fail_active_controller(self):
        logic = self.logic(["not-a-url"])
        logic.reset_search("task", "search"); logic.set_control(True, False, 0.5)
        logic.submit_frame(object()); logic.process_latest_frame()
        self.assertEqual("FAST_SWEEP", self.controller.state)
        self.assertIn("invalid_url", [event["event"] for event in self.events])


if __name__ == "__main__":
    unittest.main()
