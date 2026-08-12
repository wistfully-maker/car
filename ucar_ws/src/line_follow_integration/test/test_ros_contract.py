import hashlib
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
RUNTIME = ROOT / "src" / "line_follow_integration" / "runtime.py"

EXPECTED_SOURCES = {
    "follow_left_v4.py": "8a9471e5917b93bdddd1f0b191e8ace48fe4d6baa8269f68204d4fba23fbbfb3",
    "follow_right_v4.py": "92fd5098be423bab61c3eb5deb5c202c12adbc6c45bc4638a1af7bf5931a5d73",
    "follow_mid_v4.py": "736d0666475f25f48f1e11e888a6c40864af95496754ff37f6311fbba3d0b07e",
    "follow_left_v5.py": "d1edbf433b42181139b295f2fe31d59595ab35f26a2893ba72f6042ff03335ef",
    "follow_right_v5.py": "95d46a6cd452aa2febbfc222b0d339cdddf66ca492066747afa80478f94bd1ab",
    "yolo_server.py": "9d7010328a636742c1c60012cd6c4f0c78ae3cfc2ab4cf57c815ec88de4e9a51",
}


def _read_script(name):
    return (SCRIPTS / name).read_text(encoding="utf-8")


class SourceSnapshotTests(unittest.TestCase):
    def test_expected_sources_exist_with_exact_hashes(self):
        for name, digest in EXPECTED_SOURCES.items():
            with self.subTest(name=name):
                path = SCRIPTS / name
                self.assertTrue(path.is_file(), name)
                actual = hashlib.sha256(path.read_bytes()).hexdigest()
                self.assertEqual(digest, actual)

    def test_source_snapshot_file_records_all_hashes(self):
        snapshot = (ROOT / "SOURCE_SNAPSHOT.sha256").read_text(encoding="utf-8")
        for name, digest in EXPECTED_SOURCES.items():
            self.assertIn(name, snapshot)
            self.assertIn(digest, snapshot)
        self.assertIn(
            "4140f85c6bb2617482e47cdb6966644fcffb74f74dc213d5374f7f95bed3bbc6",
            snapshot,
        )
        self.assertIn(
            "47070a7966adcb81ff5f87abb510714023337bb6e38fa63166f3b3dc5c55041f",
            snapshot,
        )

    def test_auto_drive_and_start_all_yolo_are_not_imported(self):
        self.assertFalse((SCRIPTS / "auto_drive_v3.py").exists())
        for relative in ("launch", "scripts", "src"):
            for path in (ROOT / relative).rglob("*"):
                if path.is_file():
                    text = path.read_text(encoding="utf-8", errors="ignore")
                    self.assertNotIn(
                        "start_all_yolo.launch", text, str(path)
                    )


class V5MinimalIntegrationTests(unittest.TestCase):
    V5_SCRIPTS = ("follow_left_v5.py", "follow_right_v5.py")

    def test_v4_baseline_remains_available_and_v5_is_added(self):
        for name in (
            "follow_left_v4.py",
            "follow_right_v4.py",
            "follow_mid_v4.py",
            *self.V5_SCRIPTS,
        ):
            self.assertTrue((SCRIPTS / name).is_file(), name)

        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in self.V5_SCRIPTS:
            self.assertIn("scripts/{}".format(name), cmake)

    def test_only_left_and_right_routes_switch_to_v5(self):
        runtime = RUNTIME.read_text(encoding="utf-8")
        self.assertIn('"left_turn": "follow_left_v5.py"', runtime)
        self.assertIn('"right_turn": "follow_right_v5.py"', runtime)
        self.assertIn('"straight": "follow_mid_v4.py"', runtime)

    def test_v5_rear_line_search_is_slow_and_terminal_stop_is_ordered(self):
        for name in self.V5_SCRIPTS:
            with self.subTest(name=name):
                source = _read_script(name)
                self.assertIn("self.rear_search_speed = 0.15", source)
                self.assertIn(
                    "if self.stop_front_found:\n"
                    "                    t.linear.x = min(t.linear.x, self.rear_search_speed)",
                    source,
                )
                parking_index = source.index("if self.detect_stop_line(frame):")
                done_index = source.index(
                    'with open("/tmp/stop_done.txt","w")', parking_index
                )
                stop_index = source.index(
                    "self._publish_stop_burst()", parking_index, done_index
                )
                self.assertLess(stop_index, done_index)

    def test_v5_stops_on_shutdown_and_callback_exception(self):
        for name in self.V5_SCRIPTS:
            with self.subTest(name=name):
                source = _read_script(name)
                self.assertIn("def _publish_stop_burst(self):", source)
                self.assertIn("rospy.on_shutdown(self._publish_stop_burst)", source)
                helper_start = source.index("def _publish_stop_burst(self):")
                helper_end = source.index("\n    def ", helper_start + 1)
                helper = source[helper_start:helper_end]
                self.assertIn("time.sleep(0.03)", helper)
                self.assertNotIn("rospy.sleep", helper)
                exception_index = source.index("except Exception as e:")
                self.assertIn(
                    "self._publish_stop_burst()", source[exception_index:]
                )


class CameraAdapterContractTests(unittest.TestCase):
    def setUp(self):
        self.source = _read_script("line_camera_adapter_node.py")

    def test_subscribes_raw_line_start_and_status(self):
        for topic in (
            "/usb_cam/image_raw",
            "/task/line_follow/start",
            "/task/line_follow/status",
        ):
            self.assertIn(topic, self.source)

    def test_publishes_derived_line_image(self):
        self.assertIn("/line_follow/image_raw", self.source)

    def test_preserves_message_and_rate_limits_to_configured_fps(self):
        self.assertIn("self._publisher.publish(message)", self.source)
        self.assertIn("output_fps", self.source)

    def test_matching_frames_bypass_cv_bridge_and_image_transform(self):
        self.assertNotIn("imgmsg_to_cv2", self.source)
        self.assertNotIn("cv2_to_imgmsg", self.source)
        self.assertNotIn("transform_line_frame", self.source)

    def test_never_owns_cmd_vel_or_camera_device(self):
        self.assertNotIn("/cmd_vel", self.source)
        self.assertNotIn("VideoCapture", self.source)
        self.assertNotIn("usb_cam_node", self.source)


class NavigationAdapterContractTests(unittest.TestCase):
    def setUp(self):
        self.source = _read_script("line_navigation_adapter_node.py")

    def test_subscribes_goal_cancel_and_odom(self):
        for topic in ("/task/line_navigation_goal", "/task/cancel", "/odom"):
            self.assertIn(topic, self.source)

    def test_uses_move_base_action(self):
        self.assertIn("/move_base", self.source)
        self.assertIn("MoveBaseGoal", self.source)

    def test_publishes_arrival(self):
        self.assertIn("/task/line_navigation_arrived", self.source)

    def test_cancel_uses_task_cancel_protocol_without_goal_id(self):
        self.assertIn("parse_cancel", self.source)
        self.assertNotIn("message = parse_identity_json(raw)", self.source)


class SupervisorContractTests(unittest.TestCase):
    def setUp(self):
        self.source = _read_script("line_follow_supervisor_node.py")

    def test_subscribes_start_cancel_raw_and_derived_frames(self):
        for topic in (
            "/task/line_follow/start",
            "/task/cancel",
            "/usb_cam/image_raw",
            "/line_follow/image_raw",
        ):
            self.assertIn(topic, self.source)

    def test_candidate_velocity_source_is_subscribed(self):
        self.assertIn("/line_follow/cmd_vel_candidate", self.source)

    def test_route_child_remaps_raw_and_cmd_vel(self):
        self.assertIn("/usb_cam/image_raw:=/line_follow/image_raw", self.source)
        self.assertIn("/cmd_vel:=/line_follow/cmd_vel_candidate", self.source)

    def test_publishes_line_follow_cmd_vel_through_gate(self):
        self.assertIn("/cmd_vel/line_follow", self.source)

    def test_only_supervisor_publishes_final_line_follow_cmd_vel(self):
        for name in (
            "line_camera_adapter_node.py",
            "line_navigation_adapter_node.py",
        ):
            self.assertNotIn(
                "/cmd_vel/line_follow", _read_script(name), name
            )

    def test_only_supervisor_owns_subprocess_termination(self):
        for name in (
            "line_camera_adapter_node.py",
            "line_navigation_adapter_node.py",
        ):
            text = _read_script(name)
            self.assertNotIn("subprocess.Popen", text, name)
            self.assertNotIn("terminate", text, name)

    def test_no_broad_kill_or_tts_or_fake_success(self):
        for name in EXPECTED_SOURCES:
            text = _read_script(name)
            for forbidden in ("pkill", "killall", "rosnode kill"):
                self.assertNotIn(forbidden, text, name)

    def test_resets_both_image_gates_for_each_new_goal(self):
        self.assertIn("self._raw_gate.reset()", self.source)
        self.assertIn("self._gate.reset()", self.source)

    def test_cancel_uses_task_cancel_protocol_without_goal_id(self):
        self.assertIn("parse_cancel", self.source)

    def test_route_waits_for_a_fresh_derived_frame(self):
        self.assertIn("self._gate.allows_motion()", self.source)
        self.assertIn("self._derived_wait_started", self.source)

    def test_stale_raw_frames_block_direction_and_route_start(self):
        self.assertGreaterEqual(
            self.source.count('if raw_status != "healthy":'), 2
        )

    def test_process_liveness_uses_poll_not_container_truthiness(self):
        self.assertIn("any_process_running", self.source)
        self.assertNotIn("child_running = bool(self._children)", self.source)
        for name in (
            "line_camera_adapter_node.py",
            "line_navigation_adapter_node.py",
            "line_follow_supervisor_node.py",
        ):
            text = _read_script(name)
            for forbidden in (
                "pkill",
                "killall",
                "rosnode kill",
                "tts_http",
                "/voice/speak",
                "os.kill",
            ):
                self.assertNotIn(forbidden, text, name)


if __name__ == "__main__":
    unittest.main()
