import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent.parent


def _literal(node):
    if isinstance(node, ast.Str):
        return node.s
    if isinstance(node, ast.Num):
        return node.n
    if isinstance(node, ast.NameConstant):
        return node.value
    if hasattr(ast, "Constant") and isinstance(node, ast.Constant):
        return node.value
    raise TypeError("not a literal AST node")


class PackageConfigTest(unittest.TestCase):
    def setUp(self):
        self.scanner = (ROOT / "scripts" / "qr_scanner_node.py").read_text(encoding="utf-8")
        self.controller = (ROOT / "scripts" / "item_search_controller_node.py").read_text(encoding="utf-8")

    def test_launch_has_exact_continuous_search_parameters(self):
        root = ET.parse(str(ROOT / "launch" / "qr_item_search.launch")).getroot()
        self.assertEqual({"image_topic": "/usb_cam/image_raw"},
                         {arg.get("name"): arg.get("default") for arg in root.findall("arg")})
        nodes = {node.get("type"): node for node in root.findall("node")}
        scanner = {p.get("name"): p.get("value") for p in nodes["qr_scanner_node.py"].findall("param")}
        controller = {p.get("name"): p.get("value") for p in nodes["item_search_controller_node.py"].findall("param")}
        self.assertEqual({"image_topic": "$(arg image_topic)", "connect_timeout": "1.0",
                          "read_timeout": "2.0", "http_retries": "1", "http_worker_count": "3"}, scanner)
        self.assertEqual({"fast_angular_speed": "0.40", "targeted_angular_speed": "0.20",
                          "minimum_effective_speed": "0.11", "fast_sweep_angle": "6.632251",
                          "yaw_tolerance": "0.035", "heading_timeout": "1.0",
                          "camera_timeout": "1.0", "search_total_timeout": "40.0"}, controller)

    def test_package_runtime_dependencies_include_opencv(self):
        root = ET.parse(str(ROOT / "package.xml")).getroot()
        deps = {e.text.strip() for tag in ("depend", "exec_depend") for e in root.findall(tag)}
        self.assertTrue({"tf", "python3-pyzbar", "python3-requests", "python3-opencv",
                         "python3-numpy"}.issubset(deps))
        self.assertIn("Continuous four-edge", root.find("description").text)

    def test_nodes_use_only_new_string_protocol_topics(self):
        combined = self.scanner + self.controller
        for topic in ("/qr_item_search/scanner_event", "/qr_item_search/scanner_control",
                      "/qr_item_search/start", "/qr_item_search/stop", "/qr_item_search/result"):
            self.assertIn(topic, combined)
        for old in ("/qr_item_search/match_decision", "/qr_item_search/wall_index",
                    "/qr_item_search/scan_enabled", "/qr_item_search/reset",
                    "FrameAdapter", "StableQrDecoder", "Bool", "Empty", "Int32"):
            self.assertNotIn(old, combined)

    def test_scanner_has_decoder_thread_workers_and_shutdown(self):
        self.assertIn("thread.daemon = True", self.scanner)
        self.assertIn("run_workers(rospy.is_shutdown)", self.scanner)
        self.assertIn("rospy.on_shutdown", self.scanner)
        self.assertIn("queue_size=1", self.scanner)
        self.assertIn("buff_size=2 ** 24", self.scanner)

    def test_controller_publishers_timer_and_shutdown(self):
        tree = ast.parse(self.controller)
        self.assertIn("rospy.Duration(0.05)", self.controller)
        self.assertIn("rospy.on_shutdown", self.controller)
        cmd = self._topic_call(tree, "Publisher", "/cmd_vel")
        self.assertEqual(1, next(_literal(k.value) for k in cmd.keywords if k.arg == "queue_size"))
        for topic in ("/qr_item_search/scanner_control", "/qr_item_search/state", "/qr_item_search/result"):
            call = self._topic_call(tree, "Publisher", topic)
            self.assertEqual("String", call.args[1].id)
            self.assertIs(True, next(_literal(k.value) for k in call.keywords if k.arg == "latch"))
        for topic in ("/qr_item_search/start", "/qr_item_search/stop", "/qr_item_search/scanner_event"):
            self.assertEqual("String", self._topic_call(tree, "Subscriber", topic).args[1].id)

    def test_controller_callback_fault_stops_future_callbacks_and_shuts_down(self):
        tree = ast.parse(self.controller)
        safe = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == "main")
        nested = next(node for node in safe.body if isinstance(node, ast.FunctionDef)
                      and node.name == "safe_callback")
        calls = [node for node in ast.walk(nested) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Attribute)]
        is_set = next(node for node in calls if node.func.attr == "is_set"
                      and isinstance(node.func.value, ast.Name) and node.func.value.id == "callback_fault")
        set_call = next(node for node in calls if node.func.attr == "set"
                        and isinstance(node.func.value, ast.Name) and node.func.value.id == "callback_fault")
        shutdown = next(node for node in calls if node.func.attr == "shutdown"
                        and isinstance(node.func.value, ast.Name) and node.func.value.id == "controller")
        self.assertLess(is_set.lineno, set_call.lineno)
        self.assertLess(set_call.lineno, shutdown.lineno)
        timer = next(node for node in safe.body if isinstance(node, ast.FunctionDef)
                     and node.name == "timer_callback")
        self.assertTrue(any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                            and node.func.id == "safe_callback" for node in ast.walk(timer)))

    def test_scanner_protocol_topics_are_string(self):
        tree = ast.parse(self.scanner)
        publisher = self._topic_call(tree, "Publisher", "/qr_item_search/scanner_event")
        self.assertEqual("String", publisher.args[1].id)
        self.assertIs(True, next(_literal(k.value) for k in publisher.keywords if k.arg == "latch"))
        self.assertEqual("String", self._topic_call(tree, "Subscriber", "/qr_item_search/scanner_control").args[1].id)

    def test_scanner_publishes_uuid_session_handshake(self):
        tree = ast.parse(self.scanner)
        self.assertTrue(any(isinstance(node, ast.Import) and any(alias.name == "uuid" for alias in node.names)
                            for node in tree.body))
        self.assertIn("uuid.uuid4()", self.scanner)
        self.assertIn('"event": "scanner_started"', self.scanner)
        self.assertIn('"scanner_session": session_id', self.scanner)
        self.assertIn('value["scanner_session"] = session_id', self.scanner)

    def test_scanner_logic_construction_and_parameter_defaults(self):
        tree = ast.parse(self.scanner)
        call = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name) and node.func.id == "ScannerLogic")
        keywords = {item.arg: item.value for item in call.keywords}
        self.assertEqual("UniqueQrDecoder", keywords["decoder"].func.id)
        self.assertEqual("ItemResolver", keywords["resolver"].func.id)
        self.assertEqual("measure_quality", keywords["quality_function"].id)
        self.assertEqual("decode_variants", keywords["variant_function"].id)
        self.assertEqual("worker_count", keywords["worker_count"].id)
        self.assertEqual(3, _literal(keywords["expected_count"]))
        defaults = {}
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "get_param" and len(node.args) == 2
                    and isinstance(node.args[0], (ast.Str, ast.Constant))):
                defaults[_literal(node.args[0])] = _literal(node.args[1])
        self.assertEqual(1.0, defaults["~connect_timeout"])
        self.assertEqual(2.0, defaults["~read_timeout"])
        self.assertEqual(1, defaults["~http_retries"])
        self.assertEqual(3, defaults["~http_worker_count"])

    def test_scanner_thread_worker_shutdown_and_stop_recheck_structure(self):
        tree = ast.parse(self.scanner)
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
        self.assertTrue(any(isinstance(node.func, ast.Attribute) and node.func.attr == "run_workers" for node in calls))
        self.assertTrue(any(isinstance(node.func, ast.Attribute) and node.func.attr == "on_shutdown" for node in calls))
        self.assertTrue(any(isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Attribute) and target.attr == "daemon" for target in node.targets)
            and _literal(node.value) is True for node in ast.walk(tree)))
        clear_at = self.scanner.index("decoder_event.clear()")
        recheck_at = self.scanner.index("if stop_event.is_set() or rospy.is_shutdown():")
        process_at = self.scanner.index("logic.process_latest_frame()")
        self.assertLess(clear_at, recheck_at)
        self.assertLess(recheck_at, process_at)
        shutdown = self.scanner[self.scanner.index("def shutdown():"):]
        self.assertLess(shutdown.index("stop_event.set()"), shutdown.index("logic.set_control"))
        self.assertLess(shutdown.index("logic.set_control"), shutdown.index("decoder_event.set()"))

    def test_scanner_control_has_strict_version_and_empty_identity_rules(self):
        self.assertIn('type(value.get("protocol_version")) is not int', self.scanner)
        self.assertIn('(not task_id) != (not search_id)', self.scanner)
        self.assertIn('current_identity[:] = [None, None]', self.scanner)

    def test_cmake_still_installs_both_scripts_and_launch(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for value in ("scripts/qr_scanner_node.py", "scripts/item_search_controller_node.py",
                      "install(DIRECTORY launch/"):
            self.assertIn(value, cmake)

    @staticmethod
    def _topic_call(tree, method, topic):
        matches = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                   and isinstance(n.func, ast.Attribute) and n.func.attr == method
                   and len(n.args) >= 2 and isinstance(n.args[0], (ast.Str, ast.Constant))
                   and _literal(n.args[0]) == topic]
        if len(matches) != 1: raise AssertionError((method, topic, len(matches)))
        return matches[0]


if __name__ == "__main__":
    unittest.main()
