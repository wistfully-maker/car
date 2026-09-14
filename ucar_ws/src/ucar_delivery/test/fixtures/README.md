# test fixtures 说明

本目录只存放**脱敏、小尺寸、可复现**的合成关键帧夹具，规则如下：

1. **禁止提交**：原始大图、含人员/环境敏感信息的图片、车辆部署现场照片、
   构建缓存与 bag 文件。需要真实画面时用 `test_frame_detector.py` 中
   `make_scene()` / `full_scene()` 的确定性 numpy 合成场景代替。
2. 合成场景以代码内联（见 `test_frame_detector.py`、`test_staging_pose_estimator.py`），
   保证任何环境都能离线重跑，不依赖相机硬件。
3. 若未来确需图片夹具，只允许：
   - 分辨率 ≤ 320×240 的脱敏裁剪帧；
   - 不含人脸、车牌、文字、人员与环境标识；
   - 文件小于 50KB，并在本 README 中登记来源、用途与脱敏方式。
