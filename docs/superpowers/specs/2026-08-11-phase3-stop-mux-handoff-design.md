# Phase 3 stop mux handoff design

## Problem

Phase 2 ends by publishing `IDLE` on `/stop/motion_mode`. The Phase 3 line-start
adapter sends a new `/move_base` goal, but it does not re-enable the stop stack's
internal navigation source. Consequently, `/cmd_vel/stop_navigation` is blocked
before it reaches `/cmd_vel/stop`, even though the global arbiter is already in
`STOP_NAVIGATION`.

## Approved minimal design

`line_navigation_adapter_node.py` owns the stop stack's internal mode only for
the lifetime of a Phase 3 line-start navigation goal:

1. Publish `NAVIGATION` on `/stop/motion_mode` immediately before sending the
   accepted goal to `/move_base`.
2. Publish `IDLE` after settled success, action failure, send failure, timeout,
   cancellation, replacement failure, or ROS shutdown.
3. Keep the existing outer `/task/motion_mode` state machine unchanged.
4. Do not add a permanent global-to-stop mode bridge, because that would force
   `NAVIGATION` during Phase 2 manual parking and break its `MANUAL` ownership.

The adapter's mode publisher is not latched. The Phase 2 mission is already a
latched publisher on this topic, so adding a second latched source would make
delivery order ambiguous after a mux restart. If the mux is not connected when
Phase 3 requests navigation, the vehicle therefore remains stopped. Every mode
transition remains fail-closed: the stop mux emits zero velocity on a mode
change, and terminal paths always request `IDLE`.

## Verification

A fake-ROS node test must prove that `NAVIGATION` is published before the
`move_base` goal and that all terminal paths return to `IDLE`. Existing Phase 3,
task orchestrator, stop mux, Python compilation, XML parsing, and whitespace
checks must remain green before deployment.
