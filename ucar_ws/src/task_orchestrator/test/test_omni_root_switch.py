import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMPETITION = ROOT / "launch" / "competition_full.launch"

CONFIG_LOAD = "$(find task_orchestrator)/config/omni_fast_nav_teb.yaml"
APPLIER_TYPE = "omni_teb_applier_node.py"


class RootNavigationModeSwitchTests(unittest.TestCase):
    def _root(self):
        return ET.parse(COMPETITION).getroot()

    def _groups(self):
        root = self._root()
        groups = []
        for g in root.iter("group"):
            cond = g.attrib.get("if") or g.attrib.get("unless")
            groups.append((cond, g))
        return groups

    def test_declares_navigation_mode_default_differential(self):
        args = {
            a.attrib["name"]: a.attrib.get("default")
            for a in self._root().findall("arg")
        }
        self.assertEqual("differential", args["navigation_mode"])

    def test_supervisor_groups_forward_navigation_mode(self):
        for cond, group in self._groups():
            if cond != "$(arg start_navigation_handoff)":
                continue
            for child in group.findall("group"):
                if "if" not in child.attrib or "unless" not in child.attrib:
                    continue
                include = child.find("include")
                self.assertEqual(
                    "$(arg orchestrator_launch)", include.attrib["file"]
                )
                values = {
                    a.attrib["name"]: a.attrib["value"]
                    for a in include.findall("arg")
                }
                self.assertEqual(
                    "$(arg navigation_mode)", values["legacy_nav_mode"]
                )
                self.assertEqual(
                    "$(arg navigation_mode)",
                    values["stop_integration_nav_mode"],
                )

    def test_non_handoff_omni_group_injects_applier(self):
        matched = [
            (cond, group)
            for cond, group in self._groups()
            if cond and "navigation_mode" in cond
            and "start_navigation_handoff" in cond
        ]
        self.assertEqual(1, len(matched))
        cond, group = matched[0]
        self.assertIn("== 'omni'", cond)
        self.assertIn("== 'false'", cond)
        files = {r.attrib["file"] for r in group.iter("rosparam")}
        self.assertEqual({CONFIG_LOAD}, files)
        nodes = list(group.iter("node"))
        self.assertEqual(1, len(nodes))
        self.assertEqual(APPLIER_TYPE, nodes[0].attrib["type"])
        params = {
            p.attrib["name"]: p.attrib["value"]
            for p in nodes[0].findall("param")
        }
        self.assertEqual(
            "/omni_fast_nav/pickup_teb", params["config_path"]
        )

    def test_default_differential_never_starts_applier_in_handoff_mode(self):
        for cond, group in self._groups():
            if cond != "$(arg start_navigation_handoff)":
                continue
            self.assertFalse(any(
                n.attrib.get("type") == APPLIER_TYPE
                for n in group.iter("node")
            ))


if __name__ == "__main__":
    unittest.main()
