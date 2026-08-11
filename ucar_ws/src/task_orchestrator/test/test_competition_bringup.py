import hashlib
import unittest
import xml.etree.ElementTree as ET
import os
import shutil
import stat
import subprocess
import tempfile
from pathlib import Path

import yaml


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
        # Windows 无符号链接特权时回退为包装脚本：shebang 指向真实 bash 的
        # 绝对路径，由真实 bash 以正确 msys 根 exec 真实工具原位执行。
        real_bash = shutil.which("bash")
        for tool in ("bash", "awk", "cat", "grep", "sed", "od", "stat", "id"):
            target = shutil.which(tool)
            if not target:
                continue
            try:
                os.symlink(target, self.bin / tool)
            except OSError:
                if real_bash is None:
                    raise
                wrapper = self.bin / tool
                wrapper.write_text(
                    "#!%s\nexec \"%s\" \"$@\"\n"
                    % (real_bash.replace("\\", "/"), target.replace("\\", "/")),
                    encoding="utf-8",
                )
                wrapper.chmod(wrapper.stat().st_mode | stat.S_IXUSR)
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
            "BASE_DEVICE": "/dev/ttyS0",
            "LIDAR_DEVICE": "/dev/ttyS4",
            "CAMERA_DEVICE": "/dev/video0",
            "SPEECH_DEVICE": "/dev/ttyS3",
        }
        for name, default in defaults.items():
            script_text = script_text.replace(
                f'{name}="{default}"', f'{name}="{self.devices[name]}"',
            )
        self.script.write_text(
            script_text, encoding="utf-8", newline="\n"
        )
        self.ros_setup = self.root / "ros_setup.bash"
        self.workspace_setup = self.root / "workspace_setup.bash"
        self.ros_setup.write_text(":\n", encoding="utf-8", newline="\n")
        self.workspace_setup.write_text(":\n", encoding="utf-8", newline="\n")
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
        self._install_stop_package()
        self._install_phase3_model()

    def _install_phase3_model(self):
        """模拟第三阶段 YOLO 模型：存在且哈希与环境覆盖值一致。"""
        self.phase3_model = self.root / "yolo_model" / "best.pt"
        self.phase3_model.parent.mkdir(parents=True)
        model_bytes = b"fake-yolo-model"
        self.phase3_model.write_bytes(model_bytes)
        self.phase3_model_sha256 = hashlib.sha256(model_bytes).hexdigest()
        self.phase3_model_missing = False

    def _install_stop_package(self):
        """模拟 stop 包：模型文件 + 快照清单，供 preflight 模型校验使用。"""
        self.stop_root = self.root / "stop"
        models = self.stop_root / "scripts" / "models"
        models.mkdir(parents=True)
        model_bytes = {
            "ppocrv4_det.rknn": b"fake-det-model",
            "ppocrv4_rec.rknn": b"fake-rec-model",
            "ppocr_keys_v1.txt": b"fake-keys\n",
        }
        manifest = ["# Source: fake stop package"]
        for name, content in model_bytes.items():
            (models / name).write_bytes(content)
            manifest.append(
                "%s  scripts/models/%s"
                % (hashlib.sha256(content).hexdigest(), name)
            )
        (self.stop_root / "VEHICLE_SNAPSHOT.sha256").write_text(
            "\n".join(manifest) + "\n", encoding="utf-8", newline="\n"
        )
        self._write_fake(
            "rospack",
            'echo "rospack:$*" >> "$CALL_LOG"\n'
            'if [[ "$1 $2" == "find stop" ]]; then\n'
            '  echo "%s"\n  exit 0\nfi\nexit 1'
            % str(self.stop_root).replace("\\", "/"),
        )
        self._write_fake(
            "sha256sum",
            'echo "sha256sum:$*" >> "$CALL_LOG"\nexec /usr/bin/sha256sum "$@"',
        )

    def _write_fake(self, name, body):
        path = self.bin / name
        path.write_text(
            "#!/usr/bin/env bash\n" + body + "\n",
            encoding="utf-8", newline="\n",
        )
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
            "YOLO_MODEL": str(self.phase3_model),
            "YOLO_MODEL_SHA256": self.phase3_model_sha256,
        })
        env.update(extra_env or {})
        # 模拟小车上的 POSIX 路径，避免 Windows 反斜杠触发 coreutils 转义。
        for key in ("YOLO_MODEL", "SPARK_SECRET_FILE"):
            if key in env:
                env[key] = env[key].replace("\\", "/")
        return subprocess.run(
            [working_bash(), str(self.script), *args], env=env,
            text=True, capture_output=True, timeout=10,
            encoding="utf-8", errors="replace",
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
        self.assertNotIn("start_delivery", args)

    def test_declares_handoff_and_stop_stack_switches(self):
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertEqual("true", args["start_navigation_handoff"])
        self.assertEqual("true", args["start_stop_stack"])
        self.assertIn("$(arg start_navigation_handoff)", self.text)
        self.assertIn("start_stop_stack", self.text)

    def test_declares_phase3_line_follow_switch(self):
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertEqual("true", args["start_line_follow"])
        self.assertIn("$(arg start_line_follow)", self.text)
        self.assertIn(
            "$(find line_follow_integration)/launch/phase3.launch", self.text
        )
        self.assertIn(
            "$(find line_follow_integration)/config/phase3.yaml", self.text
        )

    def test_includes_phase3_launch_exactly_once_and_never_yolo_launch(self):
        includes = [include.attrib.get("file", "")
                    for include in self.root.iter("include")]
        self.assertEqual(1, includes.count("$(arg line_follow_launch)"))
        self.assertNotIn("start_all_yolo.launch", self.text)

    def test_phase3_group_forwards_config_path(self):
        group = next(
            group for group in self.root.findall("group")
            if group.attrib.get("if") == "$(arg start_line_follow)"
        )
        include = group.find("include")
        self.assertEqual("$(arg line_follow_launch)", include.attrib["file"])
        values = {arg.attrib["name"]: arg.attrib["value"]
                  for arg in include.findall("arg")}
        self.assertEqual("$(arg phase3_config)", values["phase3_config"])

    def test_root_launch_forwards_phase3_config_and_outer_timeouts(self):
        include = next(
            include for include in self.root.iter("include")
            if include.attrib.get("file") == "$(arg orchestrator_launch)"
        )
        values = {arg.attrib["name"]: arg.attrib["value"]
                  for arg in include.findall("arg")}
        self.assertEqual("$(arg phase3_config)", values["phase3_config"])
        self.assertEqual("$(arg timeout_line_navigation)",
                         values["timeout_line_navigation"])
        self.assertEqual("$(arg timeout_line_direction)",
                         values["timeout_line_direction"])
        self.assertEqual("$(arg timeout_line_follow)",
                         values["timeout_line_follow"])

    def test_hardware_and_navigation_owners_stay_single(self):
        self.assertEqual(1, self.text.count('<node pkg="usb_cam"'))
        self.assertEqual(
            1,
            self.text.count(
                '<include file="$(find ucar_controller)/launch/base_driver.launch"/>'
            ),
        )
        self.assertNotIn("map_server", self.text)
        self.assertNotIn("amcl", self.text)

    def test_qr_keeps_raw_image_and_line_uses_derived_topic(self):
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertEqual("/usb_cam/image_raw", args["qr_image_topic"])
        phase3 = (ROOT.parent / "line_follow_integration"
                  / "config" / "phase3.yaml")
        config = yaml.safe_load(phase3.read_text(encoding="utf-8"))
        self.assertEqual("/usb_cam/image_raw", config["camera"]["input_topic"])
        self.assertEqual("/line_follow/image_raw", config["camera"]["line_topic"])

    def test_navigation_stacks_are_only_supervisor_owned(self):
        includes = [include.attrib.get("file", "")
                    for include in self.root.iter("include")]
        self.assertFalse(any("mission_integration" in f for f in includes))
        self.assertFalse(any("legacy_navigation_include" in f for f in includes))

    def test_supervisor_group_forwards_handoff_launches(self):
        group = next(
            g for g in self.root.findall("group")
            if g.attrib.get("if") == "$(arg start_navigation_handoff)"
            and len(g.findall("group")) == 2
            and any("unless" in child.attrib for child in g.findall("group"))
        )
        enabled_group = next(g for g in group.findall("group") if "if" in g.attrib)
        disabled_group = next(g for g in group.findall("group") if "unless" in g.attrib)
        self.assertEqual("$(arg start_stop_stack)", enabled_group.attrib["if"])
        self.assertEqual("$(arg start_stop_stack)", disabled_group.attrib["unless"])
        for child, expected_stop_launch in (
            (enabled_group, "$(arg stop_integration_launch)"),
            (disabled_group, ""),
        ):
            include = child.find("include")
            self.assertEqual("$(arg orchestrator_launch)", include.attrib["file"])
            values = {a.attrib["name"]: a.attrib["value"]
                      for a in include.findall("arg")}
            self.assertEqual("true", values["enable_navigation_handoff_supervisor"])
            self.assertEqual("$(arg legacy_nav_launch)", values["legacy_nav_launch"])
            self.assertEqual(expected_stop_launch, values["stop_integration_launch"])
        defaults = {a.attrib["name"]: a.attrib.get("default")
                    for a in self.root.findall("arg")}
        self.assertIn("mission_integration.launch",
                      defaults["stop_integration_launch"])
        self.assertIn("legacy_navigation_include.launch",
                      defaults["legacy_nav_launch"])

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

    def test_base_and_lidar_are_root_owned_in_handoff_mode(self):
        # 交接模式下公共硬件由根 launch 一次性启动；导航进程组不含硬件，
        # fast-nav 只在非交接模式直接启动。
        handoff = next(
            group for group in self.root.findall("group")
            if group.attrib.get("if") == "$(arg start_navigation_handoff)"
        )
        hardware = [
            group for group in handoff.findall("group")
            if group.attrib.get("if") in
            ("$(arg start_base)", "$(arg start_lidar)")
        ]
        self.assertEqual(2, len(hardware))
        files = {
            include.attrib["file"]
            for group in hardware
            for include in group.findall("include")
        }
        self.assertEqual(
            {
                "$(find ucar_controller)/launch/base_driver.launch",
                "$(find ydlidar)/launch/ydlidar.launch",
            },
            files,
        )
        outer = next(
            group for group in self.root.findall("group")
            if group.attrib.get("if") ==
            "$(eval arg('start_navigation_handoff') == 'false')"
        )
        legacy_nav = next(
            group for group in outer.findall("group")
            if group.attrib.get("if") == "$(arg start_fast_nav)"
        )
        self.assertEqual(
            "$(eval arg('start_navigation_handoff') == 'false')",
            next(
                g.attrib["if"] for g in self.root.findall("group")
                if legacy_nav in list(g)
            ),
        )

    def test_continuous_two_workshop_phase_is_enabled_and_forwarded(self):
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertEqual("true", args["simulation_phase_enabled"])
        orchestrator_include = next(
            include for group in self.root.findall("group")
            if group.attrib.get("if") == "$(arg start_orchestrator)"
            for include in group.findall("include")
        )
        forwarded = {arg.attrib["name"]: arg.attrib["value"]
                     for arg in orchestrator_include.findall("arg")}
        self.assertEqual(
            "$(arg simulation_phase_enabled)",
            forwarded["simulation_phase_enabled"],
        )

    def test_has_one_shared_camera_and_remaps_qr_velocity(self):
        cameras = [node for node in self.root.iter("node")
                   if node.attrib.get("pkg") == "usb_cam"]
        self.assertEqual(1, len(cameras))
        self.assertEqual("log", cameras[0].attrib.get("output"))
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
        self.assertEqual(["$(arg qr_image_topic)"],
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
        for banned in ("ucar_waypoint_nav", "dynamic_obstacle",
                       "/cmd_vel/avoidance", "/cmd_vel/line"):
            self.assertNotIn(banned, self.text)
        # 配送阶段运行时切换 AMCL 合法（avoid.cpp switchToAmcl()），但本 launch
        # 不得直接定义 amcl 节点。
        self.assertNotIn('pkg="amcl"', self.text)
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
                     "enable_velocity_arbiter",
                     "enable_navigation_handoff_supervisor"):
            self.assertEqual("false", args[name])
        nodes = {node.attrib["name"]: node for node in root.findall("node")}
        self.assertEqual("$(arg enable_fast_nav_adapter)",
                         nodes["fast_nav_adapter"].attrib.get("if"))
        self.assertEqual("$(arg enable_readiness_gate)",
                         nodes["readiness_gate"].attrib.get("if"))
        self.assertEqual("$(arg enable_velocity_arbiter)",
                         nodes["velocity_arbiter"].attrib.get("if"))
        self.assertEqual("$(arg enable_navigation_handoff_supervisor)",
                         nodes["navigation_handoff_supervisor"].attrib.get("if"))

    def test_global_qr_parameters_have_exact_defaults(self):
        defaults = {
            "qr_image_topic": "/usb_cam/image_raw",
            "qr_start_debug_stream": "false",
            "qr_debug_host": "0.0.0.0",
            "qr_debug_port": "8080",
            "qr_metrics_dir": "$(env HOME)/qr_metrics",
            "qr_keyframe_dir": "$(env HOME)/qr_keyframes",
            "qr_decode_scale": "1.5",
            "qr_step_angle_deg": "45.0",
            "qr_cruise_angular_speed": "0.50",
            "qr_approach_angular_speed": "0.20",
            "qr_approach_zone_deg": "10.0",
            "qr_yaw_tolerance_deg": "2.0",
            "qr_settled_angular_speed": "0.03",
            "qr_settled_duration": "0.20",
            "qr_scan_window": "1.0",
            "qr_offset_angle_deg": "22.5",
            "qr_max_passes": "2",
            "qr_search_total_timeout": "90.0",
            "qr_settling_timeout": "3.0",
            "qr_heading_timeout": "1.0",
            "qr_camera_timeout": "1.0",
        }
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        for name, default in defaults.items():
            self.assertEqual(default, args[name], name)

    def test_qr_include_forwards_every_global_parameter(self):
        qr_groups = [group for group in self.root.findall("group")
                     if group.attrib.get("if") == "$(arg start_qr)"]
        self.assertEqual(1, len(qr_groups))
        forwarded = {arg.attrib["name"]: arg.attrib["value"]
                     for arg in qr_groups[0].find("include").findall("arg")}
        downstream = {
            "image_topic": "qr_image_topic",
            "start_debug_stream": "qr_start_debug_stream",
            "debug_host": "qr_debug_host",
            "debug_port": "qr_debug_port",
            "metrics_dir": "qr_metrics_dir",
            "keyframe_dir": "qr_keyframe_dir",
            "decode_scale": "qr_decode_scale",
            "step_angle_deg": "qr_step_angle_deg",
            "cruise_angular_speed": "qr_cruise_angular_speed",
            "approach_angular_speed": "qr_approach_angular_speed",
            "approach_zone_deg": "qr_approach_zone_deg",
            "yaw_tolerance_deg": "qr_yaw_tolerance_deg",
            "settled_angular_speed": "qr_settled_angular_speed",
            "settled_duration": "qr_settled_duration",
            "scan_window": "qr_scan_window",
            "offset_angle_deg": "qr_offset_angle_deg",
            "max_passes": "qr_max_passes",
            "search_total_timeout": "qr_search_total_timeout",
            "settling_timeout": "qr_settling_timeout",
            "heading_timeout": "qr_heading_timeout",
            "camera_timeout": "qr_camera_timeout",
        }
        for downstream_arg, global_arg in downstream.items():
            self.assertEqual("$(arg %s)" % global_arg,
                             forwarded[downstream_arg], downstream_arg)
        self.assertEqual(21, len(forwarded))

    def test_llm_launch_parameters_are_forwarded(self):
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertEqual(
            "https://spark-api-open.xf-yun.com/x2/chat/completions",
            args["llm_url"],
        )
        self.assertEqual("90.0", args["llm_request_timeout"])
        llm_groups = [group for group in self.root.findall("group")
                      if group.attrib.get("if") == "$(arg start_llm)"]
        self.assertEqual(1, len(llm_groups))
        forwarded = {arg.attrib["name"]: arg.attrib["value"]
                     for arg in llm_groups[0].find("include").findall("arg")}
        self.assertEqual("$(arg llm_url)", forwarded["url"])
        self.assertEqual("$(arg llm_request_timeout)",
                         forwarded["request_timeout"])

    def test_orchestrator_stage_timeouts_are_forwarded(self):
        defaults = {
            "timeout_dependency_ready": "120.0",
            "timeout_pickup_navigation": "300.0",
            "timeout_qr_search": "150.0",
            "timeout_llm_classification": "120.0",
            "timeout_speech": "60.0",
        }
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        for name, default in defaults.items():
            self.assertEqual(default, args[name], name)
        orchestrator_groups = [
            group for group in self.root.findall("group")
            if group.attrib.get("if") == "$(arg start_orchestrator)"
        ]
        self.assertEqual(1, len(orchestrator_groups))
        forwarded = {arg.attrib["name"]: arg.attrib["value"]
                     for arg in orchestrator_groups[0].find("include").findall("arg")}
        for name in defaults:
            self.assertEqual("$(arg %s)" % name, forwarded[name], name)
        self.assertGreater(float(args["timeout_qr_search"]),
                           float(args["qr_search_total_timeout"]))

    def test_global_launch_does_not_duplicate_hardware_owners(self):
        self.assertEqual(
            1, len([node for node in self.root.iter("node")
                    if node.attrib.get("pkg") == "usb_cam"]),
        )
        for owner in ("/map_server", "/lidar_loc", "/move_base",
                      "/base_driver", "/ydlidar_node"):
            self.assertNotIn('name="%s"' % owner, self.text)

    def test_no_delivery_auto_start_in_this_phase(self):
        self.assertNotIn("$(find ucar_avoid)", self.text)
        self.assertNotIn("amcl_delivery.launch", self.text)
        self.assertNotIn("delivery_launch", self.text)
        self.assertNotIn("start_delivery", self.text)
        args = {arg.attrib["name"] for arg in self.root.findall("arg")}
        self.assertNotIn("start_delivery", args)
        self.assertNotIn("/vision_node", self.text)
        self.assertNotIn("/racecar_control", self.text)
        self.assertNotIn("/cmd_vel/avoidance", self.text)

    def test_start_script_does_not_own_delivery_nodes(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("start_delivery", source)
        self.assertNotIn("/vision_node", source)
        self.assertNotIn("/racecar_control", source)

    def test_start_script_rejects_all_phase3_owned_nodes(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            "conflicts+=(/line_camera_adapter /line_navigation_adapter "
            "/line_follow_supervisor /phase3_yolo_server)",
            source,
        )
        self.assertIn("/phase3_line_follower_*", source)

    def test_start_script_static_safety_contract(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        for required in (
            "ROS_SETUP", "WORKSPACE_SETUP", "SPARK_API_PASSWORD",
            "rosnode list", "rosnode ping", "rostopic info /cmd_vel",
            "exec roslaunch task_orchestrator competition_full.launch",
            "/dev/ttyS0", "/dev/ttyS4", "/dev/ttyS3",
        ):
            self.assertIn(required, source)
        for forbidden in (
            "rosnode kill", "kill -9", "pkill", "killall", "rm ",
        ):
            self.assertNotIn(forbidden, source)
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/start_competition.sh", cmake)

    def test_start_script_retains_posix_executable_bit(self):
        mode = subprocess.check_output(
            ["git", "-C", str(ROOT), "ls-files", "-s", "--",
             str(START_SCRIPT)],
            text=True,
        ).split()[0]
        self.assertEqual(
            "100755", mode,
            "start_competition.sh must be executable in the Git archive",
        )

    def test_production_device_paths_are_fixed_and_probe_is_fail_closed(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        for assignment in (
            'BASE_DEVICE="/dev/ttyS0"',
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
        self.assertIn("Publishers:[ \\t][ \\t]*None", source)
        self.assertIn("Subscribers:[ \\t][ \\t]*None", source)
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

    def test_start_script_case_patterns_use_explicit_continuations(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        argument_case = source.split('case "$key" in', 1)[1].split("esac", 1)[0]
        for line in argument_case.splitlines():
            if line.rstrip().endswith("|"):
                self.fail("Bash case pattern must not end a physical line: %s" % line)

    def test_start_script_boolean_whitelist_is_exactly_start_switches(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        argument_case = source.split('case "$key" in', 1)[1].split("esac", 1)[0]
        patterns = set()
        for line in argument_case.splitlines():
            stripped = line.strip().rstrip("|)\\").strip()
            if not stripped or stripped == ";;":
                continue
            if "normalise_bool" in stripped or stripped.startswith("#"):
                continue
            if stripped.startswith("*)"):
                continue
            for part in stripped.split("|"):
                part = part.strip()
                if part:
                    patterns.add(part)
        for pattern in patterns:
            self.assertRegex(pattern, r"^start_[a-z_]+$", pattern)
        self.assertEqual(
            {
                "start_fast_nav", "start_base", "start_lidar",
                "start_camera", "start_fast_nav_adapter",
                "start_readiness_gate", "start_speech", "start_qr",
                "start_llm", "start_orchestrator", "start_velocity_arbiter",
                "start_navigation_handoff", "start_stop_stack",
                "start_line_follow",
            },
            patterns,
        )

    def test_start_script_declares_handoff_contract(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("[start_navigation_handoff]=true", source)
        self.assertIn("[start_stop_stack]=true", source)
        self.assertIn("start_navigation_handoff|start_stop_stack", source)
        self.assertIn("conflicts+=(/navigation_handoff_supervisor)", source)
        self.assertIn("rospack find stop", source)
        self.assertIn("VEHICLE_SNAPSHOT.sha256", source)
        self.assertIn("sha256sum", source)
        self.assertNotIn("eval ", source)

    def test_start_script_keeps_all_layer_security_contracts(self):
        source = START_SCRIPT.read_text(encoding="utf-8")
        for conflict in (
            "conflicts+=(/usb_cam)",
            "conflicts+=(/speech_command_node)",
            "conflicts+=(/qr_scanner /item_search_controller)",
            "conflicts+=(/spark_llm_node)",
            "conflicts+=(/task_orchestrator /voice_task_adapter /tts_bridge)",
            "conflicts+=(/fast_nav_adapter)",
            "conflicts+=(/readiness_gate)",
            "conflicts+=(/velocity_arbiter)",
            "conflicts+=(/map_server /lidar_loc /move_base)",
            "conflicts+=(/base_driver)",
            "conflicts+=(/ydlidar_node)",
        ):
            self.assertIn(conflict, source)
        for guard in (
            "live node conflict",
            "stale ROS master registration",
            "requires /lidar_loc",
            "/cmd_vel already has a direct publisher",
            "external arbiter mode requires exactly one /cmd_vel publisher",
            "exec roslaunch task_orchestrator competition_full.launch",
        ):
            self.assertIn(guard, source)
        self.assertNotIn("start_delivery", source)


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

    def test_phase3_owned_and_anonymous_child_nodes_are_conflicts(self):
        for node in (
            "/line_camera_adapter",
            "/line_navigation_adapter",
            "/line_follow_supervisor",
            "/phase3_yolo_server",
            "/phase3_line_follower_123_456",
        ):
            with self.subTest(node=node):
                _, result = self.run_fake(
                    nodes=node, live_nodes={node},
                )
                self.assertNotEqual(0, result.returncode)
                self.assertIn("live node conflict", result.stderr.lower())

    def test_rejects_amcl_when_lidar_loc_is_internal_or_external(self):
        for args in (("start_fast_nav:=true",),
                     ("start_fast_nav:=false",
                      "start_navigation_handoff:=false")):
            with self.subTest(args=args):
                _, result = self.run_fake(
                    *args, nodes="/amcl", live_nodes={"/amcl"}
                )
                self.assertNotEqual(0, result.returncode)
                self.assertIn("/amcl", result.stderr)

    def test_external_fastnav_does_not_claim_its_nodes_or_devices(self):
        fake, result = self.run_fake(
            "start_fast_nav:=false", "start_navigation_handoff:=false",
            nodes="/lidar_loc", live_nodes={"/lidar_loc"},
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

    def test_handoff_switches_validate_booleans(self):
        for arg in ("start_navigation_handoff:=maybe", "start_stop_stack:=maybe"):
            with self.subTest(arg=arg):
                _, result = self.run_fake(arg, master=False)
                self.assertNotEqual(0, result.returncode)
                self.assertIn("invalid boolean", result.stderr.lower())

    def test_clean_boot_with_handoff_and_stop_stack_switches(self):
        _, result = self.run_fake(
            "start_navigation_handoff:=false", "start_stop_stack:=false",
            "start_fast_nav:=false", master=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)

    def test_valid_stop_models_pass_preflight(self):
        fake, result = self.run_fake(master=False)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("rospack:", fake.calls())

    def test_phase3_switch_strictly_validates_boolean(self):
        _, result = self.run_fake("start_line_follow:=maybe", master=False)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("invalid boolean", result.stderr.lower())

    def test_phase3_model_passes_preflight(self):
        fake, result = self.run_fake(master=False)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "sha256sum:%s" % str(fake.phase3_model).replace("\\", "/"),
            fake.calls(),
        )

    def test_phase3_model_missing_is_fatal(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        fake.phase3_model.unlink()
        result = fake.run()
        self.assertNotEqual(0, result.returncode)
        self.assertIn("yolo model", result.stderr.lower())

    def test_phase3_model_sha_mismatch_is_fatal(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        result = fake.run(extra_env={"YOLO_MODEL_SHA256": "0" * 64})
        self.assertNotEqual(0, result.returncode)
        self.assertIn("mismatch", result.stderr.lower())

    def test_disabling_phase3_skips_only_model_check(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        fake.phase3_model.unlink()
        result = fake.run("start_line_follow:=false")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertNotIn("yolo model", result.stderr.lower())

    def test_phase3_tuning_args_are_forwarded_verbatim(self):
        fake, result = self.run_fake(
            "start_line_follow:=true", "timeout_line_follow:=130.0",
            "line_follow_launch:=/custom/phase3.launch", master=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        for arg in (
            "start_line_follow:=true",
            "timeout_line_follow:=130.0",
            "line_follow_launch:=/custom/phase3.launch",
        ):
            self.assertIn("roslaunch-arg:%s" % arg, fake.calls())

    def test_missing_stop_model_file_is_fatal(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        (fake.stop_root / "scripts/models/ppocrv4_det.rknn").unlink()
        result = fake.run()
        self.assertNotEqual(0, result.returncode)
        self.assertIn("missing stop model", result.stderr.lower())

    def test_stop_model_manifest_mismatch_is_fatal(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        (fake.stop_root / "scripts/models/ppocrv4_det.rknn").write_bytes(b"tampered")
        result = fake.run()
        self.assertNotEqual(0, result.returncode)
        self.assertIn("manifest mismatch", result.stderr.lower())

    def test_stop_model_check_is_skipped_when_stop_stack_external(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        (fake.stop_root / "scripts/models/ppocrv4_det.rknn").unlink()
        result = fake.run("start_stop_stack:=false")
        self.assertEqual(0, result.returncode, result.stderr)

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
        self.assertIn("roslaunch-arg:competition_full.launch", fake.calls())
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
        secret.write_text(
            "$(touch %s)\n" % marker, encoding="utf-8", newline="\n"
        )
        secret.chmod(0o600)
        if os.name == "posix":
            # Windows 文件系统没有 Unix 权限位（stat 恒报 0666），
            # 0600 通过断言只在 POSIX 上可验证。
            result = fake.run(extra_env={
                "SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret),
            })
            self.assertEqual(0, result.returncode, result.stderr)
        else:
            result = fake.run(extra_env={
                "SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret),
            })
            self.assertNotEqual(0, result.returncode)
            self.assertIn("permissions", result.stderr.lower())
        self.assertFalse(marker.exists())
        secret.write_text("first\nsecond\n", encoding="utf-8", newline="\n")
        result = fake.run(extra_env={
            "SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret),
        })
        self.assertNotEqual(0, result.returncode)
        if os.name == "posix":
            self.assertIn("single line", result.stderr.lower())

    def test_secret_file_metadata_and_crlf_are_rejected(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        secret = fake.root / "secret"
        secret.write_text("literal-secret\n", encoding="utf-8", newline="\n")
        secret.chmod(0o644)
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret)})
        self.assertNotEqual(0, result.returncode)
        if os.name == "posix":
            secret.chmod(0o600)
            result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret)})
            self.assertEqual(0, result.returncode, result.stderr)
        secret.write_bytes(b"literal-secret\r\n")
        result = fake.run(extra_env={"SPARK_API_PASSWORD": "", "SPARK_SECRET_FILE": str(secret)})
        self.assertNotEqual(0, result.returncode)
        link = fake.root / "secret-link"
        try:
            link.symlink_to(secret)
        except OSError:
            # Windows 无符号链接特权：跳过符号链接子场景。
            return
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
        prefix = "roslaunch-arg:"
        recorded = [line[len(prefix):] for line in fake.calls().splitlines()
                    if line.startswith("roslaunch-arg:")]
        self.assertEqual(["task_orchestrator", "competition_full.launch", *arguments], recorded)

    def test_tuning_parameters_are_forwarded_verbatim(self):
        fake = FakeRosEnvironment(self, master=False)
        self.addCleanup(fake.close)
        arguments = (
            "qr_step_angle_deg:=30.0", "qr_scan_window:=0.8",
            "qr_search_total_timeout:=120.0", "llm_request_timeout:=45.0",
            "timeout_qr_search:=180.0", "qr_cruise_angular_speed:=0.60",
        )
        result = fake.run(*arguments)
        self.assertEqual(0, result.returncode, result.stderr)
        prefix = "roslaunch-arg:"
        recorded = [line[len(prefix):] for line in fake.calls().splitlines()
                    if line.startswith("roslaunch-arg:")]
        self.assertEqual(
            ["task_orchestrator", "competition_full.launch", *arguments],
            recorded,
        )
        self.assertNotIn("normalise_bool", fake.calls())


if __name__ == "__main__":
    unittest.main()
