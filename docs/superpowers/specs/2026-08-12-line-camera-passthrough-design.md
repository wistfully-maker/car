# Line Camera Passthrough Design

## Problem

The vehicle camera publishes `/usb_cam/image_raw` as `640x480`, `rgb8`, at about 30 FPS. The line camera adapter successfully decodes and transforms a frame, but the vehicle's Noetic `cv_bridge.cv2_to_imgmsg(..., "bgr8")` raises `KeyError: 16`. Consequently `/line_follow/image_raw` never becomes ready and phase 3 fails before route execution.

## Design

Keep the shared camera and QR configuration unchanged. While a line-follow goal is active, accept only frames whose dimensions already match the configured `640x480` output and publish the original ROS `Image` message to `/line_follow/image_raw` at the configured 15 FPS. Do not decode, crop, resize, recolor, or re-encode the image. The route scripts already request `bgr8` from `CvBridge`, so retaining the source `rgb8` encoding preserves their expected OpenCV input.

If the source dimensions do not match the configured output, fail closed by logging a throttled warning and publishing no derived frame. This avoids silently changing the field of view or image geometry.

## Verification

- Unit/contract tests prove the adapter uses passthrough and never calls `cv2_to_imgmsg`.
- Existing line-follow tests remain green.
- On the vehicle, `/line_follow/image_raw` must report `640x480`, `rgb8`, near 15 FPS during an active goal.
- No camera or velocity command is published by diagnostic verification.
