# QR README Operations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a complete field-operations and integration README for the existing QR search package.

**Architecture:** Documentation-only change. Preserve current ROS behavior and describe implemented commands separately from future integration contracts.

**Tech Stack:** Markdown, ROS 1 Noetic, `roslaunch`, `rostopic`, UTF-8 JSON over `std_msgs/String`.

---

### Task 1: Rewrite the package README

**Files:**
- Modify: `ucar_ws/src/qr_item_search/README.md`

- [ ] **Step 1: Record current runtime behavior**

Document the existing launch file, nodes, topics, parameters and terminal states from source.

- [ ] **Step 2: Add executable operating procedures**

Add exact commands for build, prerequisite checks, package launch, manual start, stop, observation, node shutdown and scanner-only testing.

- [ ] **Step 3: Add future integration procedures**

Document the `task1_orchestrator` boundary and exact JSON examples for voice, arrival, QR, LLM, TTS and navigation simulation.

- [ ] **Step 4: Verify the document**

Run UTF-8 decoding, forbidden-placeholder scan, command/topic consistency checks and the existing package test suite.

- [ ] **Step 5: Commit**

```bash
git add docs/superpowers/specs/2026-07-22-qr-readme-operations-design.md docs/superpowers/plans/2026-07-22-qr-readme-operations.md ucar_ws/src/qr_item_search/README.md
git commit -m "docs: add QR operations and integration guide"
```

