"""Phase E: lock the three-sided white parking frame detector contract.

Synthetic numpy scenes cover far/medium/close geometry, moderate yaw,
lateral offset, missing front boundary, partial side boundaries, glare/noise,
no-frame and loss/reacquisition. Input images must not be modified.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import frame_detector
from ucar_delivery.frame_detector import FrameConfirmGate, WhiteFrameDetector

WIDTH = 640
HEIGHT = 480
FLOOR = 60
LINE = 255
CENTER_X = WIDTH / 2.0


def make_scene(
    width=WIDTH,
    height=HEIGHT,
    floor=FLOOR,
    line=LINE,
    near_span=None,
    near_y=None,
    far_span=None,
    far_y=None,
    left_x=None,
    right_x=None,
    noise=0.0,
):
    image = np.full((height, width), floor, dtype=np.uint8)
    if near_span is not None and near_y is not None:
        image[near_y[0]:near_y[1], near_span[0]:near_span[1]] = line
    if far_span is not None and far_y is not None:
        image[far_y[0]:far_y[1], far_span[0]:far_span[1]] = line
    if left_x is not None:
        image[:, left_x[0]:left_x[1]] = line
    if right_x is not None:
        image[:, right_x[0]:right_x[1]] = line
    if noise > 0.0:
        rng = np.random.default_rng(7)
        mask = rng.random((height, width)) < noise
        image[mask] = 255
    return image


def full_scene(**overrides):
    scene = dict(
        near_span=(300, 500),
        near_y=(440, 452),
        far_span=(360, 540),
        far_y=(300, 306),
        left_x=(140, 148),
        right_x=(492, 500),
    )
    scene.update(overrides)
    return make_scene(**scene)


DEFAULT_CONFIG = {
    "gray_threshold": 180,
    "min_line_pixels": 60,
    "min_col_pixels": 60,
}


class Harness:
    def __init__(self, config=None):
        merged = dict(DEFAULT_CONFIG)
        merged.update(config or {})
        self.detector = WhiteFrameDetector(merged)
        self.now = [1000.0]

    def detect(self, image):
        obs = self.detector.detect(image, self.now[0])
        self.now[0] += 0.1
        return obs


class FullFrameTests(unittest.TestCase):
    def test_full_frame_detected_with_all_boundaries(self):
        h = Harness()
        obs = h.detect(full_scene())
        self.assertTrue(obs.frame_detected)
        self.assertEqual(4, obs.visible_boundary_count)
        self.assertAlmostEqual(1.0, obs.confidence)
        self.assertAlmostEqual(400.0, obs.near_center_x, delta=2.0)
        self.assertAlmostEqual(450.0, obs.far_center_x, delta=2.0)
        self.assertAlmostEqual(144.0, obs.left_boundary, delta=2.0)
        self.assertAlmostEqual(496.0, obs.right_boundary, delta=2.0)
        self.assertAlmostEqual(303.0, obs.front_boundary_y, delta=2.0)

    def test_timestamp_preserved(self):
        h = Harness()
        obs = h.detect(full_scene())
        self.assertEqual(1000.0, obs.timestamp)

    def test_input_image_not_modified(self):
        h = Harness()
        image = full_scene()
        before = image.copy()
        h.detect(image)
        self.assertTrue(np.array_equal(before, image))


class GeometryTests(unittest.TestCase):
    def test_medium_frame(self):
        h = Harness()
        obs = h.detect(
            full_scene(
                near_span=(320, 480),
                near_y=(420, 430),
                far_span=(370, 470),
                far_y=(260, 264),
                left_x=(160, 166),
                right_x=(474, 480),
            )
        )
        self.assertTrue(obs.frame_detected)
        self.assertAlmostEqual(400.0, obs.near_center_x, delta=2.0)

    def test_close_frame_with_front_line_leaving_image(self):
        h = Harness()
        obs = h.detect(
            full_scene(near_y=(462, 480), far_span=(380, 500), far_y=(360, 364))
        )
        self.assertTrue(obs.frame_detected)
        self.assertEqual(4, obs.visible_boundary_count)

    def test_far_frame_thin_front_line(self):
        h = Harness()
        obs = h.detect(
            full_scene(
                near_span=(340, 460),
                near_y=(430, 434),
                far_span=(390, 450),
                far_y=(150, 152),
                left_x=(180, 184),
                right_x=(456, 460),
            )
        )
        self.assertTrue(obs.frame_detected)
        self.assertAlmostEqual(420.0, obs.far_center_x, delta=2.0)

    def test_moderate_yaw_derives_center_difference(self):
        h = Harness()
        obs = h.detect(
            full_scene(near_span=(260, 460), far_span=(360, 560))
        )
        self.assertAlmostEqual(360.0, obs.near_center_x, delta=2.0)
        self.assertAlmostEqual(460.0, obs.far_center_x, delta=2.0)
        self.assertAlmostEqual(
            obs.far_center_x - obs.near_center_x, obs.yaw_error, delta=0.5
        )

    def test_lateral_offset_left_of_center(self):
        h = Harness()
        obs = h.detect(
            full_scene(near_span=(200, 400), far_span=(260, 460))
        )
        self.assertLess(obs.near_center_x, CENTER_X)
        self.assertAlmostEqual(
            obs.near_center_x - CENTER_X, obs.lateral_error, delta=0.5
        )


class DegradedFrameTests(unittest.TestCase):
    def test_missing_front_boundary(self):
        h = Harness()
        obs = h.detect(
            full_scene(far_span=None, far_y=None)
        )
        self.assertTrue(obs.frame_detected)
        self.assertEqual(3, obs.visible_boundary_count)
        # 0.6（3 边界基础）+ 0.1（左右边线包围近线）
        self.assertAlmostEqual(0.70, obs.confidence)
        self.assertIsNone(obs.far_center_x)
        self.assertIsNone(obs.front_boundary_y)

    def test_missing_right_side_boundary(self):
        h = Harness()
        obs = h.detect(full_scene(right_x=None))
        self.assertTrue(obs.frame_detected)
        self.assertEqual(3, obs.visible_boundary_count)
        self.assertIsNone(obs.right_boundary)

    def test_missing_left_side_boundary(self):
        h = Harness()
        obs = h.detect(full_scene(left_x=None))
        self.assertTrue(obs.frame_detected)
        self.assertEqual(3, obs.visible_boundary_count)
        self.assertIsNone(obs.left_boundary)

    def test_noise_and_glare_tolerated(self):
        h = Harness()
        obs = h.detect(full_scene(noise=0.005))
        self.assertTrue(obs.frame_detected)
        self.assertEqual(4, obs.visible_boundary_count)
        self.assertAlmostEqual(400.0, obs.near_center_x, delta=3.0)


class NoFrameTests(unittest.TestCase):
    def test_floor_only_no_frame(self):
        h = Harness()
        obs = h.detect(make_scene())
        self.assertFalse(obs.frame_detected)
        self.assertEqual(0, obs.visible_boundary_count)
        self.assertAlmostEqual(0.0, obs.confidence)
        self.assertIsNone(obs.near_center_x)
        self.assertIsNone(obs.far_center_x)

    def test_loss_and_reacquisition(self):
        h = Harness()
        obs = h.detect(full_scene())
        self.assertTrue(obs.frame_detected)
        obs = h.detect(make_scene())
        self.assertFalse(obs.frame_detected)
        obs = h.detect(full_scene())
        self.assertTrue(obs.frame_detected)

    def test_isolated_small_blobs_are_not_a_frame(self):
        h = Harness()
        image = make_scene()
        image[100:120, 100:130] = 255
        image[300:310, 400:420] = 255
        obs = h.detect(image)
        self.assertFalse(obs.frame_detected)


class ThresholdTests(unittest.TestCase):
    def test_low_contrast_line_rejected_at_high_threshold(self):
        h = Harness(config={"gray_threshold": 180})
        scene = full_scene(line=150)
        obs = h.detect(scene)
        self.assertFalse(obs.frame_detected)

    def test_low_contrast_line_accepted_at_low_threshold(self):
        h = Harness(config={"gray_threshold": 120})
        scene = full_scene(line=150)
        obs = h.detect(scene)
        self.assertTrue(obs.frame_detected)

    def test_side_line_minimum_length_configured(self):
        h = Harness(config={"min_col_pixels": 500})
        obs = h.detect(full_scene())
        self.assertTrue(obs.frame_detected)
        self.assertEqual(2, obs.visible_boundary_count)
        self.assertIsNone(obs.left_boundary)
        self.assertIsNone(obs.right_boundary)


class RoiTests(unittest.TestCase):
    """任务 D：ROI 外目标不得触发；ROI 内目标输出全图坐标。"""

    def roi_harness(self, roi):
        return Harness(dict(DEFAULT_CONFIG, **roi))

    def test_target_outside_roi_not_detected(self):
        h = self.roi_harness({
            "roi_y_min": 460, "roi_y_max": 480, "roi_x_min": 0,
            "roi_x_max": 640,
        })
        # 框主体在 y<460；ROI 内只有边线残留 20 行，不足 min_col_pixels
        obs = h.detect(full_scene())
        self.assertFalse(obs.frame_detected)
        self.assertEqual(0, obs.visible_boundary_count)

    def test_target_inside_roi_reports_full_image_coordinates(self):
        h = self.roi_harness({
            "roi_y_min": 120, "roi_y_max": 480, "roi_x_min": 100,
            "roi_x_max": 540,
        })
        obs = h.detect(full_scene())
        self.assertTrue(obs.frame_detected)
        # 坐标必须仍是全图坐标，而不是 ROI 相对坐标
        self.assertAlmostEqual(400.0, obs.near_center_x, delta=2.0)
        self.assertAlmostEqual(144.0, obs.left_boundary, delta=2.0)

    def test_input_image_not_modified_with_roi(self):
        h = self.roi_harness({
            "roi_y_min": 120, "roi_y_max": 480, "roi_x_min": 0,
            "roi_x_max": 640,
        })
        image = full_scene()
        before = image.copy()
        h.detect(image)
        self.assertTrue(np.array_equal(before, image))


class HsvTests(unittest.TestCase):
    """任务 D：可选 HSV 白色阈值拒绝彩色杂物，不改变输入图像。"""

    def make_color_scene(self, lines, clutter):
        image = np.full((480, 640, 3), (60, 60, 90), dtype=np.uint8)
        for (y0, y1, x0, x1) in lines:
            image[y0:y1, x0:x1] = (255, 255, 255)
        for (y0, y1, x0, x1) in clutter:
            image[y0:y1, x0:x1] = (255, 0, 0)
        return image

    def full_color_scene(self):
        return self.make_color_scene(
            lines=[
                (440, 452, 300, 500),
                (300, 306, 360, 540),
                (0, 480, 140, 148),
                (0, 480, 492, 500),
            ],
            clutter=[(200, 240, 200, 260)],
        )

    def test_hsv_filter_rejects_colorful_clutter(self):
        h = Harness(dict(DEFAULT_CONFIG, use_hsv=True, hsv_sat_max=60.0))
        obs = h.detect(self.full_color_scene())
        self.assertTrue(obs.frame_detected)
        self.assertEqual(4, obs.visible_boundary_count)
        # 红色杂物被饱和度过滤，不产生伪边界
        self.assertNotIn(230.0, [obs.left_boundary, obs.right_boundary])

    def test_hsv_clutter_only_region_not_detected(self):
        h = Harness(dict(DEFAULT_CONFIG, use_hsv=True, hsv_sat_max=60.0))
        image = self.make_color_scene(
            lines=[],
            clutter=[(200, 240, 200, 260), (300, 310, 400, 420)],
        )
        obs = h.detect(image)
        self.assertFalse(obs.frame_detected)


class MorphologyTests(unittest.TestCase):
    """任务 D：开闭运算去噪：孤立盐噪声被清除，断线缺口被弥合。"""

    def test_open_removes_salt_noise(self):
        h = Harness(dict(DEFAULT_CONFIG, morph_iterations=1))
        image = full_scene()
        # 在框内撒 40 个孤立白点（不足以形成段）
        rng = np.random.default_rng(3)
        ys, xs = rng.integers(200, 460, 40), rng.integers(100, 540, 40)
        image[ys, xs] = 255
        obs = h.detect(image)
        self.assertTrue(obs.frame_detected)
        self.assertEqual(4, obs.visible_boundary_count)

    def test_morphology_does_not_modify_input(self):
        h = Harness(dict(DEFAULT_CONFIG, morph_iterations=1))
        image = full_scene()
        before = image.copy()
        h.detect(image)
        self.assertTrue(np.array_equal(before, image))


class GeometryConsistencyTests(unittest.TestCase):
    """任务 D：错误横线/竖线组合、大片白色、透视违反不得高置信通过。"""

    def test_side_lines_on_same_side_rejected(self):
        h = Harness()
        obs = h.detect(
            full_scene(left_x=(480, 488), right_x=(492, 500))
        )
        self.assertLess(obs.confidence, 0.65)
        self.assertFalse(obs.frame_detected)

    def test_side_lines_not_enclosing_near_line_rejected(self):
        h = Harness()
        # 左边线在近线中心右侧 → 组合不自洽
        obs = h.detect(full_scene(left_x=(420, 428)))
        self.assertLess(obs.confidence, 0.65)
        self.assertFalse(obs.frame_detected)

    def test_thick_white_block_penalized(self):
        h = Harness()
        image = full_scene(near_span=(300, 500), near_y=(400, 480))
        obs = h.detect(image)
        self.assertLess(obs.confidence, 0.65)
        self.assertFalse(obs.frame_detected)

    def test_overbright_glare_rejected(self):
        h = Harness()
        image = np.full((480, 640), 255, dtype=np.uint8)
        obs = h.detect(image)
        self.assertFalse(obs.frame_detected)

    def test_perspective_violation_penalized(self):
        h = Harness()
        # 远端比近端宽 50%：违反近宽远窄的透视一致性
        obs = h.detect(
            full_scene(near_span=(340, 460), far_span=(260, 540))
        )
        self.assertLess(obs.confidence, 0.65)
        self.assertFalse(obs.frame_detected)

    def test_broken_lines_still_detected(self):
        h = Harness()
        image = full_scene()
        # 近线中间断 8 像素（gap 容差内），远线断 4 像素
        image[440:452, 390:398] = FLOOR
        image[300:306, 440:444] = FLOOR
        obs = h.detect(image)
        self.assertTrue(obs.frame_detected)
        self.assertEqual(4, obs.visible_boundary_count)

    def test_shadow_reduces_detection(self):
        h = Harness(config={"gray_threshold": 180})
        image = full_scene()
        # 阴影压低右侧亮度（不影响线，影响地板）—— 检测仍应成功
        image[:, 400:] = np.clip(image[:, 400:].astype(int) - 30, 0, 255).astype(np.uint8)
        obs = h.detect(image)
        self.assertTrue(obs.frame_detected)


class FrameConfirmGateTests(unittest.TestCase):
    """任务 D：多帧获取确认、短暂丢失宽限、近区丢失标志。"""

    def make_observation(self, detected, front_y=300.0):
        return frame_detector.FrameObservation(
            timestamp=100.0,
            frame_detected=detected,
            confidence=0.9 if detected else 0.0,
            visible_boundary_count=4 if detected else 0,
            front_boundary_y=front_y,
        )

    def test_acquire_requires_confirm_frames(self):
        gate = FrameConfirmGate({"acquire_confirm_frames": 3})
        for _ in range(2):
            obs = gate.update(self.make_observation(True))
            self.assertFalse(obs.confirmed)
        obs = gate.update(self.make_observation(True))
        self.assertTrue(obs.confirmed)

    def test_lost_requires_grace_frames(self):
        gate = FrameConfirmGate({"acquire_confirm_frames": 1,
                                 "lost_grace_frames": 2})
        self.assertTrue(gate.update(self.make_observation(True)).confirmed)
        obs = gate.update(self.make_observation(False))
        self.assertTrue(obs.confirmed)  # 宽限期第 1 帧仍确认
        obs = gate.update(self.make_observation(False))
        self.assertTrue(obs.confirmed)  # 宽限期第 2 帧仍确认
        obs = gate.update(self.make_observation(False))
        self.assertFalse(obs.confirmed)  # 超过宽限 → 丢失

    def test_short_loss_within_grace_does_not_lose(self):
        gate = FrameConfirmGate({"acquire_confirm_frames": 1,
                                 "lost_grace_frames": 2})
        gate.update(self.make_observation(True))
        gate.update(self.make_observation(False))
        obs = gate.update(self.make_observation(True))
        self.assertTrue(obs.confirmed)

    def test_near_zone_flag_and_loss(self):
        gate = FrameConfirmGate({"acquire_confirm_frames": 1,
                                 "lost_grace_frames": 2,
                                 "near_zone_front_y": 380.0})
        obs = gate.update(self.make_observation(True, front_y=400.0))
        self.assertTrue(obs.near_zone)
        self.assertFalse(obs.near_zone_loss)
        obs = gate.update(self.make_observation(False, front_y=None))
        self.assertTrue(obs.near_zone_loss)  # 近区丢框立即标志
        self.assertTrue(obs.confirmed)
        obs = gate.update(self.make_observation(False, front_y=None))
        self.assertTrue(obs.near_zone_loss)  # 宽限期内持续标志
        obs = gate.update(self.make_observation(False, front_y=None))
        self.assertFalse(obs.confirmed)  # 超过宽限 → 丢失
        obs = gate.update(self.make_observation(False, front_y=None))
        self.assertFalse(obs.near_zone_loss)  # 已确认丢失，不再标志

    def test_far_zone_loss_has_no_fatal_flag(self):
        gate = FrameConfirmGate({"acquire_confirm_frames": 1,
                                 "lost_grace_frames": 1,
                                 "near_zone_front_y": 380.0})
        gate.update(self.make_observation(True, front_y=300.0))
        obs = gate.update(self.make_observation(False, front_y=300.0))
        self.assertFalse(obs.near_zone_loss)

    def test_acquire_counter_resets_after_loss(self):
        gate = FrameConfirmGate({"acquire_confirm_frames": 3,
                                 "lost_grace_frames": 1})
        for _ in range(3):
            gate.update(self.make_observation(True))
        self.assertTrue(gate.update(self.make_observation(False)).confirmed)
        gate.update(self.make_observation(False))
        # 重新获取需要重新累计确认帧
        gate.update(self.make_observation(True))
        self.assertFalse(gate.update(self.make_observation(True)).confirmed)


if __name__ == "__main__":
    unittest.main()
