# Phase 3 V5 Line Follow Minimal Integration Implementation Plan

> **For agentic workers:** Execute inline in the existing `codex/phase3-line-follow-integration` worktree; do not create another branch or worktree.

**Goal:** Add vehicle V5 left/right line followers behind the existing supervisor while retaining V4 and minimizing final-parking risk.

**Architecture:** Import the two vehicle V5 scripts as new versioned entrypoints. Change only the left/right route table and install manifest; keep the straight route and supervisor protocol unchanged. Add small safety changes inside V5 after front-line detection and at terminal/error/shutdown paths.

**Tech Stack:** ROS1 `rospy`, Python 3, `unittest`, catkin CMake.

---

### Task 1: Lock the integration contract

**Files:**
- Modify: `ucar_ws/src/line_follow_integration/test/test_ros_contract.py`

- [ ] Assert V4 files and hashes remain present.
- [ ] Assert V5 files exist, are installed, and left/right route selection uses them while straight remains V4.
- [ ] Assert both V5 files cap rear-line search to 0.15 m/s, stop before the done marker, and register shutdown/error zero-speed protection.
- [ ] Run the focused tests and confirm they fail because V5 is not yet present.

### Task 2: Add and minimally harden V5

**Files:**
- Create: `ucar_ws/src/line_follow_integration/scripts/follow_left_v5.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/follow_right_v5.py`

- [ ] Import the exact current vehicle sources (vehicle hashes recorded in the source manifest).
- [ ] Add a repeated-zero helper and `rospy.on_shutdown` registration.
- [ ] Cap motion after front-line acceptance to 0.15 m/s.
- [ ] Publish repeated zero before creating `/tmp/stop_done.txt` and on callback exceptions.

### Task 3: Wire routes and packaging

**Files:**
- Modify: `ucar_ws/src/line_follow_integration/src/line_follow_integration/runtime.py`
- Modify: `ucar_ws/src/line_follow_integration/CMakeLists.txt`
- Modify: `ucar_ws/src/line_follow_integration/SOURCE_SNAPSHOT.sha256`
- Modify: `ucar_ws/src/line_follow_integration/README.md`

- [ ] Route left/right to V5 and leave straight on V4.
- [ ] Install V5 while continuing to install V4.
- [ ] Record upstream vehicle hashes and integrated hashes without rewriting the V4 records.
- [ ] Document rollback by changing only the two route-table values.

### Task 4: Verify

- [ ] Run focused RED/GREEN tests.
- [ ] Run all `line_follow_integration` tests.
- [ ] Run Python compilation and `git diff --check`.
- [ ] Confirm git diff contains no V4 modifications and no unrelated files.
