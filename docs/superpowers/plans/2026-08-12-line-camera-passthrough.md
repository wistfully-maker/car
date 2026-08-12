# Line Camera Passthrough Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore `/line_follow/image_raw` by directly forwarding the already-correct `640x480` ROS image at 15 FPS.

**Architecture:** The adapter remains the activation and rate-limit gate. It publishes the source `sensor_msgs/Image` unchanged when dimensions match, avoiding the broken vehicle-side ROS image re-encoding path.

**Tech Stack:** ROS Noetic, Python 3, `sensor_msgs/Image`, Python `unittest`.

---

### Task 1: Lock the passthrough contract

**Files:**
- Modify: `ucar_ws/src/line_follow_integration/test/test_ros_contract.py`

- [ ] Add assertions that the adapter publishes the input message and does not call `cv2_to_imgmsg`, `imgmsg_to_cv2`, or `transform_line_frame`.
- [ ] Run `python -m unittest ucar_ws/src/line_follow_integration/test/test_ros_contract.py -v` and verify the new test fails against the existing transform pipeline.

### Task 2: Implement the minimal passthrough

**Files:**
- Modify: `ucar_ws/src/line_follow_integration/scripts/line_camera_adapter_node.py`

- [ ] Remove OpenCV/CvBridge transformation dependencies from the adapter.
- [ ] Reject mismatched image dimensions with a throttled warning.
- [ ] Publish the original message after the existing activation and timestamp rate gates.
- [ ] Re-run the focused test and confirm it passes.

### Task 3: Regression and vehicle verification

**Files:**
- No production file beyond Task 2.

- [ ] Run the entire `line_follow_integration` test suite, `compileall`, and `git diff --check`.
- [ ] Commit only the design, plan, test, and adapter change.
- [ ] Back up the deployed adapter with a timestamp, copy the fixed file, and run `catkin_make` if required.
- [ ] Restart the competition launch under operator control, activate phase 3 through the normal flow, and verify `/line_follow/image_raw` metadata/rate without publishing motion commands.
