# Phase 3 V5 Line Follow Minimal Integration Design

## Goal

Integrate the vehicle-tested left/right V5 followers into the existing phase-3 supervisor without replacing the retained V4 baseline or changing the straight route.

## Approved scope

- Keep `follow_left_v4.py`, `follow_right_v4.py`, and `follow_mid_v4.py` byte-for-byte unchanged.
- Add local copies of the vehicle V5 left/right implementations.
- Route `left_turn` and `right_turn` to V5; keep `straight` on `follow_mid_v4.py`.
- Preserve the supervisor remaps from `/line_follow/image_raw` and to `/line_follow/cmd_vel_candidate`; V5 must not become the final `/cmd_vel` owner.
- Preserve V5's 8-second parking-detection mask and existing perception/turning constants.
- After the front parking line has been accepted, cap the rear-line search speed at 0.15 m/s.
- Before writing `/tmp/stop_done.txt`, publish repeated zero velocity commands. Also publish repeated zero commands on callback exception and ROS shutdown.
- Do not deploy or run a motion test in this local implementation step.

## Data flow

`line_follow_supervisor_node.py` selects a script from `ROUTE_SCRIPTS`, starts it with the existing image and velocity remaps, gates candidate velocity, and remains the sole publisher to `/cmd_vel/line_follow`. The global arbiter remains the sole final `/cmd_vel` publisher.

## Verification

Static contract tests lock V4 retention, V5 route selection, V5 installation, camera/velocity remaps, rear-line search speed, stop-before-marker ordering, and shutdown/exception stop protection. Existing package tests and Python compilation must remain green.
