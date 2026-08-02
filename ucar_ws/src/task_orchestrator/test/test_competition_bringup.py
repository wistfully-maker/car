import unittest
import xml.etree.ElementTree as ET
import os
import shutil
import stat
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPETITION = ROOT / "launch" / "competition_full.launch"
ORCHESTRATOR = ROOT / "launch" / "task_orchestrator.launch"
START_SCRIPT = ROOT / "scripts" / "start_competition.sh"


def working_bash():
    candidate = shutil.which("bash")
    if not candidate:
        return None
    try:
        result = subprocess.run(
            [candidate, "--version"], capture_output=True, timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return candidate if result.returncode == 0 else None


class FakeRosEnvironment:
    def __init__(self, test_case, master=True, nodes="", live_nodes=None,
                 cmd_vel_publishers="None", cmd_vel_status="ok",
                 device_status="free", fuser_available=True,
                 lsof_available=True):
        self.test_case = test_case
        self.master = master
        self.nodes = nodes
        self.live_nodes = set(live_nodes or ())
        self.cmd_vel_publishers = cmd_vel_publishers
        self.cmd_vel_status = cmd_vel_status
        self.device_status = device_status
        self.fuser_available = fuser_available
        self.lsof_available = lsof_available
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        for tool in ("bash", "awk", "grep", "sed", "od", "stat", "id"):
            target = shutil.which(tool)
            if target:
                os.symlink(target, self.bin / tool)
        self.log = self.root / "calls.log"
        self.script = self.root / "start_competition.sh"
        self.devices = {
            "BASE_DEVICE": self.root / "base-device",
            "LIDAR_DEVICE": self.root / "lidar-device",
            "CAMERA_DEVICE": self.root / "camera-device",
            "SPEECH_DEVICE": self.root / "speech-device",
        }
        for device in self.devices.values():
            device.touch()
        script_text = START_SCRIPT.read_text(encoding="utf-8")
        defaults = {
            "BASE_DEVICE": "/dev/ucar_controller",
            "LIDAR_DEVICE": "/dev/ttyS4",
            "CAMERA_DEVICE": "/dev/video0",
            "SPEECH_DEVICE": "/dev/ttyS3",
        }
        for name, default in defaults.items():
            script_text = script_text.replace(
                f'{name}="{default}"', f'{name}="{self.devices[name]}"',
            )
        self.script.write_text(script_text, encoding="utf-8")
        self.ros_setup = self.root / "ros_setup.bash"
        self.workspace_setup = self.root / "workspace_setup.bash"
        self.ros_setup.write_text(":\n", encoding="utf-8")
        self.workspace_setup.write_text(":\n", encoding="utf-8")
        self._write_fake("rosnode", self._rosnode())
        self._write_fake("rostopic", self._rostopic())
        if self.fuser_available:
            self._write_fake("fuser", self._probe("fuser"))
        if self.lsof_available:
            self._write_fake("lsof", self._probe("lsof"))
        self._write_fake(
            "roslaunch",
            'for arg in "$@"; do printf "roslaunch-arg:%s\\n" "$arg" >> "$CALL_LOG"; done\nexit 0',
        )

    def _write_fake(self, name, body):
        path = self.bin / name
        path.write_text("#!/usr/bin/env bash\n" + body + "\n", encoding="utf-8")
        path.chmod(path.stat().st_mode | stat.S_IXUSR)

    def _rosnode(self):
        live = " ".join(sorted(self.live_nodes))
        return f'''echo "rosnode:$*" >> "$CALL_LOG"
if [[ "$1" == list ]]; then
  [[ "{1 if self.master else 0}" == 1 ]] || exit 1
  printf '%s\\n' {self.nodes!r}
  exit 0
fi
if [[ "$1" == ping ]]; then
  case " {live} " in *" ${{@: -1}} "*) exit 0;; *) exit 1;; esac
fi
exit 2'''

    def _rostopic(self):
        if self.cmd_vel_status == "missing":
            result = 'echo "ERROR: Unknown topic /cmd_vel" >&2\nexit 1'
        elif self.cmd_vel_status == "error":
            result = 'echo "ERROR: Unable to communicate with master" >&2\nexit 1'
        elif self.cmd_vel_status == "malformed":
            result = 'printf "Publishers:\\n * /node (http://x/)\\n"\nexit 0'
        elif self.cmd_vel_status == "none_then_star":
            result = 'printf "Publishers: None\\n * /node (http://x/)\\nSubscribers: None\\n"\nexit 0'
        elif self.cmd_vel_status == "reversed":
            result = 'printf "Subscribers: None\\nPublishers: None\\n"\nexit 0'
        elif self.cmd_vel_status == "duplicate":
            result = 'printf "Publishers: None\\nPublishers: None\\nSubscribers: None\\n"\nexit 0'
        elif self.cmd_vel_status == "unknown_success":
            result = 'echo "Unknown topic /cmd_vel"\nexit 0'
        elif self.cmd_vel_status == "two_publishers":
            result = ('printf "Publishers:\\n * /one (http://x/)\\n'
                      ' * /two (http://y/)\\nSubscribers: None\\n"\nexit 0')
        else:
            if self.cmd_vel_publishers == "None":
                result = '''printf "Publishers: None\\nSubscribers: None\\n"
exit 0'''
            else:
                result = f'''cat <<'EOF'
Type: geometry_msgs/Twist
Publishers:
{self.cmd_vel_publishers}
Subscribers: None
EOF
exit 0'''
        return f'''echo "rostopic:$*" >> "$CALL_LOG"
if [[ "$1 $2" == "info /cmd_vel" ]]; then
{result}
fi
exit 1'''

    def _probe(self, name):
        outcomes = {
            "busy": "echo 4242; exit 0",
            "free": "exit 1",
            "permission": 'echo "permission denied" >&2; exit 1',
            "error": 'echo "probe failed" >&2; exit 2',
        }
        return f'''echo "{name}:$*" >> "$CALL_LOG"
{outcomes[self.device_status]}'''

    def run(self, *args, extra_env=None):
        env = os.environ.copy()
        env.update({
            "PATH": str(self.bin),
            "CALL_LOG": str(self.log),
            "ROS_SETUP": str(self.ros_setup),
            "WORKSPACE_SETUP": str(self.workspace_setup),
            "SPARK_API_PASSWORD": "test-secret",
        })
        env.update(extra_env or {})
        return subprocess.run(
            [working_bash(), str(self.script), *args], env=env,
            text=True, capture_output=True, timeout=10,
        )

    def calls(self):
        return self.log.read_text(encoding="utf-8") if self.log.exists() else ""

    def close(self):
        self.temp.cleanup()


class CompetitionBringupTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(COMPETITION).getroot()
        self.text = COMPETITION.read_text(encoding="utf-8")

    def test_declares_every_independent_start_switch(self):
        expected = {
            "start_fast_nav", "start_base", "start_lidar", "start_camera",
            "start_fast_nav_adapter", "start_readiness_gate", "start_speech",
            "start_qr", "start_llm", "start_orchestrator",
            "start_velocity_arbiter",
        }
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertTrue(expected <= set(args))
        for name in expected:
            self.assertIn("$(arg %s)" % name, self.text)
        self.assertEqual("true", args["start_velocity_arbiter"])

    def test_documents_external_contracts_and_fail_fast_boundaries(self):
        for marker in (
            "EXTERNAL_CONTRACT",
            "ucar_fast_nav/pickup_navigation.launch",
            "speech_command/speech_command.launch",
            "qr_item_search/qr_item_search.launch",
            "llm_spark/llm_spark.launch",
            "usb_cam/usb_cam_node",
            "fail fast",
            "does not start a camera",
            "external equivalent sole arbiter",
        ):
            self.assertIn(marker, self.text)

    def test_uses_real_external_launch_interfaces(self):
        includes = {include.attrib["file"]: include
                    for include in self.root.iter("include")}
        nav = includes["$(arg fast_nav_launch)"]
        self.assertEqual(
            {"start_base": "$(arg start_base)",
             "start_lidar": "$(arg start_lidar)",
             "cmd_vel_topic": "/cmd_vel/navigation"},
            {arg.attrib["name"]: arg.attrib["value"]
             for arg in nav.findall("arg")},
        )
        self.assertIn("$(find ucar_fast_nav)/launch/pickup_navigation.launch",
                      self.text)
        self.assertIn("$(find speech_command)/launch/speech_command.launch",
                      self.text)
        self.assertIn("$(find qr_item_search)/launch/qr_item_search.launch",
                      self.text)
        self.assertIn("$(find llm_spark)/launch/llm_spark.launch", self.text)

    def test_base_and_lidar_are_only_fast_nav_downstream_switches(self):
        nav_group = next(group for group in self.root.findall("group")
                         if group.attrib.get("if") == "$(arg start_fast_nav)")
        nav_args = {arg.attrib["name"]: arg.attrib["value"]
                    for arg in nav_group.find("include").findall("arg")}
        self.assertEqual("$(arg start_base)", nav_args["start_base"])
        self.assertEqual("$(arg start_lidar)", nav_args["start_lidar"])
        outside = "".join(ET.tostring(group, encoding="unicode")
                          for group in self.root.findall("group")
                          if group is not nav_group)
        self.assertNotIn("$(arg start_base)", outside)
        self.assertNotIn("$(arg start_lidar)", outside)

    def test_has_one_shared_camera_and_remaps_qr_velocity(self):
        cameras = [node for node in self.root.iter("node")
                   if node.attrib.get("pkg") == "usb_cam"]
        self.assertEqual(1, len(cameras))
        qr_groups = [group for group in self.root.findall("group")
                     if group.attrib.get("if") == "$(arg start_qr)"]
        self.assertEqual(1, len(qr_groups))
        qr = qr_groups[0]
        self.assertEqual(
            {"/cmd_vel": "/cmd_vel/qr"},
            {r.attrib["from"]: r.attrib["to"] for r in qr.findall("remap")},
        )
        image_args = [arg for arg in qr.iter("arg")
                      if arg.attrib.get("name") == "image_topic"]
        self.assertEqual(["/usb_cam/image_raw"],
                         [arg.attrib.get("value") for arg in image_args])

    def test_delegates_waypoint_loading_exclusively_to_vendor_launch(self):
        waypoint = [p for p in self.root.iter("rosparam")
                    if p.attrib.get("ns") == "/ucar_fast_nav"]
        self.assertEqual([], waypoint)
        nav_includes = [include for include in self.root.iter("include")
                        if include.attrib.get("file") == "$(arg fast_nav_launch)"]
        self.assertEqual(1, len(nav_includes))
        defaults = {arg.attrib["name"]: arg.attrib.get("default")
                    for arg in self.root.findall("arg")}
        self.assertEqual(
            "$(find ucar_fast_nav)/launch/pickup_navigation.launch",
            defaults["fast_nav_launch"],
        )
        self.assertNotIn("pickup_goal:", self.text)

    def test_starts_auxiliary_nodes_with_private_full_config(self):
        expected = {
            "fast_nav_adapter": "fast_nav_adapter_node.py",
            "readiness_gate": "system_readiness_gate_node.py",
            "velocity_arbiter": "velocity_arbiter_node.py",
        }
        task_root = ET.parse(ORCHESTRATOR).getroot()
        nodes = {node.attrib.get("name"): node for node in task_root.findall("node")}
        for name, executable in expected.items():
            self.assertEqual(executable, nodes[name].attrib.get("type"))
            rosparam = nodes[name].find("rosparam")
            self.assertEqual("load", rosparam.attrib.get("command"))
            self.assertEqual("$(find task_orchestrator)/config/orchestrator.yaml",
                             rosparam.attrib.get("file"))
        enabled = {
            "fast_nav_adapter": ("start_fast_nav_adapter", "enable_fast_nav_adapter"),
            "readiness_gate": ("start_readiness_gate", "enable_readiness_gate"),
            "velocity_arbiter": ("start_velocity_arbiter", "enable_velocity_arbiter"),
        }
        for _, (start_arg, enable_arg) in enabled.items():
            groups = [g for g in self.root.findall("group")
                      if g.attrib.get("if") == "$(arg %s)" % start_arg]
            self.assertEqual(1, len(groups))
            values = {a.attrib["name"]: a.attrib["value"]
                      for a in groups[0].find("include").findall("arg")}
            self.assertEqual("true", values[enable_arg])

    def test_bans_legacy_navigation_and_extra_velocity_outputs(self):
        for banned in ("ucar_waypoint_nav", "amcl", "dynamic_obstacle",
                       "/cmd_vel/avoidance", "/cmd_vel/line"):
            self.assertNotIn(banned, self.text)
        self.assertEqual(1, self.text.count('to="/cmd_vel/qr"'))
        self.assertIn('value="/cmd_vel/navigation"', self.text)
        self.assertEqual("true", next(
            arg.attrib["default"] for arg in self.root.findall("arg")
            if arg.attrib["name"] == "start_velocity_arbiter"
        ))

    def test_task_launch_auxiliary_nodes_are_opt_in(self):
        root = ET.parse(ORCHESTRATOR).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        for name in ("enable_fast_nav_adapter", "enable_readiness_gate",
                     "enable_velocity_arbiter"):
            self.assertEqual("false", args[name])
        nodes = {node.attrib["name"]: node for node in root.findall("node")}
        self.assertEqual("$(arg enable_fast_nav_adapter)",
                         nodes["fast_nav_adapter"].attrib.get("if"))
        self.assertEqual("$(arg enable_readiness_gate)",
                         nodes["readiness_gate"].attrib.get("if"))
        self.assertEqual("$(arg enable_velocity_arbiter)",
                         nodes["velocity_arbiter"].attrib.get("if"))

    def test_start_script_static_safety_contract(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        for required in (
            "ROS_SETUP", "WORKSPACE_SETUP", "SPARK_API_PASSWORD",
            "rosnode list", "rosnode ping", "rostopic info /cmd_vel",
            "exec roslaunch task_orchestrator competition_full.launch",
            "/dev/ucar_controller", "/dev/ttyS4", "/dev/ttyS3",
        ):
            self.assertIn(required, source)
        for forbidden in (
            "rosnode kill", "kill -9", "pkill", "killall", "rm ",
        ):
            self.assertNotIn(forbidden, source)
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/start_competition.sh", cmake)

    def test_production_device_paths_are_fixed_and_probe_is_fail_closed(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        for assignment in (
            'BASE_DEVICE="/dev/ucar_controller"',
            'LIDAR_DEVICE="/dev/ttyS4"',
            'CAMERA_DEVICE="/dev/video0"',
            'SPEECH_DEVICE="/dev/ttyS3"',
        ):
            self.assertIn(assignment, source)
        self.assertNotIn('${BASE_DEVICE:-', source)
        self.assertNotIn('${LIDAR_DEVICE:-', source)
        self.assertNotIn('${CAMERA_DEVICE:-', source)
        self.assertNotIn('${SPEECH_DEVICE:-', source)
        for marker in ("probe_rc", "probe_stderr", "install fuser or lsof"):
            self.assertIn(marker, source)

    def test_secret_metadata_and_topic_output_are_fail_closed(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        for marker in (
            '[[ ! -L "$SPARK_SECRET_FILE" ]]',
            '[[ -f "$SPARK_SECRET_FILE" ]]',
            'stat -c', "id -u", "secret_mode", "Publishers:", "Subscribers:",
            "malformed rostopic info /cmd_vel",
        ):
            self.assertIn(marker, source)

    def test_topic_parser_accepts_ros_inline_none_contract(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("Publishers:[[:space:]]+None", source)
        self.assertIn("Subscribers:[[:space:]]+None", source)
        self.assertIn("publisher_none", source)

    def test_start_script_uses_vendor_runtime_node_contract(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        # Source: vendor runtime_check.sh and navigation_full.launch handoff.
        for node in (
            "/base_driver", "/ydlidar_node", "/map_server",
            "/lidar_loc", "/move_base",
        ):
            self.assertIn(node, source)
        self.assertNotIn("conflicts+=(/ydlidar)", source)
        self.assertIn('[[ "${flags[start_base]}" == true ]] && conflicts+=(/base_driver)', source)
        self.assertIn('[[ "${flags[start_lidar]}" == true ]] && conflicts+=(/ydlidar_node)', source)
        self.assertIn("conflicts+=(/map_server /lidar_loc /move_base)", source)

    def test_secret_file_is_parsed_as_data_and_keys_are_whitelisted(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn('source "$SPARK_SECRET_FILE"', source)
        self.assertNotIn('eval ', source)
        self.assertIn('SPARK_API_PASSWORD=', source)
        self.assertIn('case "$key" in', source)
        self.assertNotIn('[[ -v "flags[', source)


@unittest.skipUnless(working_bash(), "no working Bash available")
class CompetitionStartScriptTests(unittest.TestCase):
    def run_fake(self, *args, **kwargs):
        fake = FakeRosEnvironment(self, **kwargs)
        self.addCleanup(fake.close)
        return fake, fake.run(*args)

    def test_clean_boot_reaches_roslaunch_and_preserves_arguments(self):
        fake, result = self.run_fake(
            "start_camera:=false", "unknown_arg:=kept", master=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "roslaunch-arg:task_orchestrator\n"
            "roslaunch-arg:competition_full.launch\n"
            "roslaunch-arg:start_camera:=false\n"
            "roslaunch-arg:unknown_arg:=kept", fake.calls(),
        )
        self.assertNotIn("rostopic:", fake.calls())

    def test_rejects_invalid_boolean(self):
        _, result = self.run_fake("start_lidar:=maybe", master=False)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("invalid boolean", result.stderr.lower())

    def test_reports_live_and_stale_node_conflicts(self):
        for live, word in ((True, "live"), (False, "stale")):
            with self.subTest(live=live):
                _, result = self.run_fake(
                    nodes="/usb_cam", live_nodes={"/usb_cam"} if live else set(),
                )
                self.assertNotEqual(0, result.returncode)
                self.assertIn(word, result.stderr.lower())

    def test_rejects_amcl_when_lidar_loc_is_internal_or_external(self):
        for arg in ("start_fast_nav:=true", "start_fast_nav:=false"):
            with self.subTest(arg=arg):
                _, result = self.run_fake(arg, nodes="/amcl", live_nodes={"/amcl"})
                self.assertNotEqual(0, result.returncode)
                self.assertIn("/amcl", result.stderr)

    def test_external_fastnav_does_not_claim_its_nodes_or_devices(self):
        fake, result = self.run_fake(
            "start_fast_nav:=false", nodes="/lidar_loc", live_nodes={"/lidar_loc"},
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertNotIn(str(fake.devices["BASE_DEVICE"]), fake.calls())
        self.assertNotIn(str(fake.devices["LIDAR_DEVICE"]), fake.calls())

    def test_device_conflict_is_fatal(self):
        _, result = self.run_fake(device_status="busy")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("device", result.stderr.lower())

    def test_device_probe_matrix_is_fail_closed(self):
        fake, result = self.run_fake(device_status="free")
        self.assertEqual(0, result.returncode, result.stderr)
        fake, result = self.run_fake(device_status="permission")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("probe", result.stderr.lower())
        fake, result = self.run_fake(device_status="error")
        self.assertNotEqual(0, result.returncode)
        fake, result = self.run_fake(
            device_status="free", fuser_available=False, lsof_available=True,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("lsof:", fake.calls())
        _, result = self.run_fake(
            fuser_available=False, lsof_available=False,
        )
        self.assertNotEqual(0, result.returncode)

    def test_missing_owned_device_is_fatal(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        fake.devices["CAMERA_DEVICE"].unlink()
        result = fake.run()
        self.assertNotEqual(0, result.returncode)
        self.assertIn("absent", result.stderr.lower())

    def test_missing_secret_is_only_required_for_llm(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(fake.root / "missing")})
        self.assertNotEqual(0, result.returncode)
        self.assertIn("spark", result.stderr.lower())
        result = fake.run("start_llm:=false", extra_env={"SPARK_API_PASSWORD": ""})
        self.assertEqual(0, result.returncode, result.stderr)

    def test_arbiter_ownership_rules(self):
        _, result = self.run_fake(cmd_vel_publishers=" * /old_arbiter (http://x/)")
        self.assertNotEqual(0, result.returncode)
        _, result = self.run_fake(
            "start_velocity_arbiter:=false", master=False,
        )
        self.assertNotEqual(0, result.returncode)
        _, result = self.run_fake(
            "start_velocity_arbiter:=false", "start_fast_nav:=false",
            cmd_vel_publishers=" * /external_arbiter (http://x/)",
        )
        self.assertEqual(0, result.returncode, result.stderr)

    def test_missing_cmd_vel_is_zero_owners_but_transport_error_is_fatal(self):
        fake, result = self.run_fake(cmd_vel_status="missing")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("roslaunch:", fake.calls())
        _, result = self.run_fake(
            "start_velocity_arbiter:=false", cmd_vel_status="missing",
        )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("exactly one", result.stderr)
        _, result = self.run_fake(cmd_vel_status="error")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("communicate", result.stderr.lower())

    def test_malformed_successful_cmd_vel_info_is_fatal(self):
        for status in (
            "malformed", "none_then_star", "reversed", "duplicate",
            "unknown_success",
        ):
            with self.subTest(status=status):
                _, result = self.run_fake(cmd_vel_status=status)
                self.assertNotEqual(0, result.returncode)
                self.assertIn("malformed", result.stderr.lower())

    def test_legal_ros_topic_layouts_count_publishers(self):
        fake, result = self.run_fake(cmd_vel_status="ok")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("roslaunch-arg:competition_full.launch", fake.calls())
        _, result = self.run_fake(
            "start_velocity_arbiter:=false", cmd_vel_status="two_publishers",
        )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("found 2", result.stderr)

    def test_secret_file_is_literal_single_line_data(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        marker = fake.root / "must_not_exist"
        secret = fake.root / "secret"
        secret.write_text("$(touch %s)\n" % marker, encoding="utf-8")
        secret.chmod(0o600)
        result = fake.run(extra_env={
            "SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret),
        })
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertFalse(marker.exists())
        secret.write_text("first\nsecond\n", encoding="utf-8")
        result = fake.run(extra_env={
            "SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret),
        })
        self.assertNotEqual(0, result.returncode)
        self.assertIn("single line", result.stderr.lower())

    def test_secret_file_metadata_and_crlf_are_rejected(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        secret = fake.root / "secret"
        secret.write_text("literal-secret\n", encoding="utf-8")
        secret.chmod(0o644)
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret)})
        self.assertNotEqual(0, result.returncode)
        secret.chmod(0o600)
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret)})
        self.assertEqual(0, result.returncode, result.stderr)
        secret.write_bytes(b"literal-secret\r\n")
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret)})
        self.assertNotEqual(0, result.returncode)
        link = fake.root / "secret-link"
        link.symlink_to(secret)
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(link)})
        self.assertNotEqual(0, result.returncode)

    def test_unknown_malicious_key_is_only_forwarded(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        marker = fake.root / "must_not_exist"
        argument = "x[$(touch %s)]:=true" % marker
        result = fake.run(argument)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertFalse(marker.exists())
        self.assertIn("roslaunch-arg:" + argument, fake.calls())

    def test_unknown_argument_boundaries_are_preserved(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        marker = fake.root / "must_not_exist"
        arguments = ("unknown:=two words", "glob:=*", "code:=$(touch %s)" % marker)
        result = fake.run(*arguments)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertFalse(marker.exists())
        recorded = [line.removeprefix("roslaunch-arg:") for line in fake.calls().splitlines()
                    if line.startswith("roslaunch-arg:")]
        self.assertEqual(["task_orchestrator", "competition_full.launch", *arguments], recorded)


if __name__ == "__main__":
    unittest.main()
