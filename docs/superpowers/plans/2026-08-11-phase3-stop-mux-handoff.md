# Phase 3 Stop Mux Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow the Phase 3 line-start `/move_base` goal to pass through the stop stack's internal velocity mux without changing Phase 2 behavior.

**Architecture:** The existing line navigation adapter temporarily owns `/stop/motion_mode` while its goal is active. It selects `NAVIGATION` immediately before `send_goal` and restores `IDLE` on every terminal path; no new node or global mode bridge is introduced.

**Tech Stack:** ROS Noetic `rospy`, `actionlib`, `std_msgs/String`, Python `unittest`

---

### Task 1: Lock the internal stop-mode lifecycle

**Files:**
- Create: `ucar_ws/src/line_follow_integration/test/test_line_navigation_adapter_node.py`
- Modify: `ucar_ws/src/line_follow_integration/scripts/line_navigation_adapter_node.py`

- [ ] **Step 1: Write the failing node tests**

Create a fake ROS publisher registry and action client. Assert that a valid line
goal publishes `NAVIGATION` before the action client's `send_goal`, and that
settled success, non-success action result, send exception, cancellation,
timeout, and shutdown each leave the final internal mode at `IDLE`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_line_navigation_adapter_node.py" -v
```

Expected: failures because the adapter does not create `/stop/motion_mode` and
does not publish `NAVIGATION` or terminal `IDLE`.

- [ ] **Step 3: Implement the minimal mode publisher**

Add a latched `std_msgs/String` publisher for `/stop/motion_mode`, a small
`_set_stop_mode(mode)` helper, `NAVIGATION` immediately before `send_goal`, and
`IDLE` in every path that clears the active navigation identity.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the command from Step 2. Expected: all new tests pass with no warnings.

- [ ] **Step 5: Run regression and static verification**

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/stop/test -p "test_velocity_mux*.py" -v
python -m compileall -q ucar_ws/src/line_follow_integration ucar_ws/src/task_orchestrator
git diff --check
```

Expected: zero failures and exit code 0 for every command.

- [ ] **Step 6: Commit only the scoped files**

```powershell
git add docs/superpowers/specs/2026-08-11-phase3-stop-mux-handoff-design.md docs/superpowers/plans/2026-08-11-phase3-stop-mux-handoff.md ucar_ws/src/line_follow_integration/test/test_line_navigation_adapter_node.py ucar_ws/src/line_follow_integration/scripts/line_navigation_adapter_node.py
git commit -m "fix(phase3): enable stop navigation for line handoff"
```
