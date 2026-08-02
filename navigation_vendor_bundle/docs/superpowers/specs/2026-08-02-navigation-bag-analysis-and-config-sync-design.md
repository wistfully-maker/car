# Navigation Bag Analysis and Vehicle Configuration Sync Design

## Context

The `ucar_fast_nav` vehicle run completes the route but exhibits wall contact,
straight-line yaw oscillation, repeated probing, and poor omnidirectional cornering.
Read-only inspection of `/home/ucar/ucar_ws/src/ucar_fast_nav` and
`ucar_fast_wall_contact.bag` established that the vehicle configuration differs
from the development snapshot and that the existing bag analyzer is insufficient
to separate localization, planning, control, and chassis-execution faults.

The first implementation phase will improve diagnostics and synchronize the
development snapshot. It will not deploy files to the vehicle or tune runtime
parameters on the vehicle.

## Confirmed Evidence

- The recorded runtime used GlobalPlanner and TEB.
- `/scan` ran at about 10.06 Hz and `/odom` at about 19.98 Hz.
- `/cmd_vel` ran at about 5.16 Hz despite a configured 15 Hz controller loop.
- TEB local-plan publication ran at about 4.64 Hz, with a 95th-percentile gap of
  about 0.51 s and a maximum gap of about 1.45 s.
- Forward commands peaked at about 0.35 m/s, while angular commands repeatedly
  saturated at `+/-1.2 rad/s`.
- Consecutive `map -> odom` samples showed large discontinuities: about 0.108 m
  translation and 3.4 degrees yaw at the 95th percentile, with maxima of about
  0.422 m and 11 degrees.
- The vehicle uses a `0.328 x 0.244 m` costmap footprint, but TEB uses a point
  footprint with `min_obstacle_dist: 0.15`.
- The effective costmap inflation radius is `0.05 m`; the apparent `0.25 m`
  local override is in the wrong namespace.

## Goals

1. Make bag analysis produce evidence for each reported behavior rather than
   only listing bins with a short laser range.
2. Keep analysis logic testable on the Windows development host without ROS.
3. Preserve compatibility with ROS Noetic `rosbag` when the script is streamed
   to or run on the vehicle.
4. Synchronize the development navigation snapshot with the exact configuration
   observed on the vehicle and update bundle provenance/hash metadata.
5. Establish repeatable local checks before any parameter-tuning experiment.

## Non-goals

- No SSH writes, package installation, node startup, or parameter changes on the
  vehicle.
- No claim that a parameter change fixes wall contact without a new vehicle bag.
- No planner replacement or broad navigation-stack rewrite.
- No committed copy of the 3.9 MB vehicle bag.

## Analyzer Architecture

`tools/analyze_fast_wall_bag.py` will be split internally into two layers while
remaining a single executable script:

1. A pure-Python analysis layer receives normalized timestamped samples and
   computes metrics. It must not import ROS modules.
2. A ROS bag adapter lazily imports `rosbag`, converts supported messages into
   normalized samples, and feeds the pure layer.

This boundary allows local unit tests to construct synthetic samples without a
ROS installation. The command-line interface remains compatible with the current
usage:

```text
python3 analyze_fast_wall_bag.py BAG
```

Existing `--bin` and `--near` arguments remain supported. Geometry and diagnostic
thresholds receive explicit options with vehicle defaults, including footprint
half-length/half-width, laser offset, angular saturation threshold, localization
jump thresholds, and command-staleness threshold.

## Analyzer Outputs

The default human-readable report will contain these sections:

1. Bag and topic coverage, including required missing topics.
2. Per-topic count, effective rate, median/95th/max gap.
3. Command statistics for `vx`, `vy`, and `wz`, including saturation, sign flips,
   reverse commands, and lateral-dominant commands.
4. Odom statistics and timestamp-aligned command/odom errors.
5. `map -> odom` translation and yaw discontinuities.
6. Global/local path pose counts, cumulative lengths, and local-plan publication
   gaps.
7. Near-wall events paired by timestamp rather than combining an arbitrary
   minimum scan with the last command in a bin.
8. Laser-to-body clearance estimates using the configured laser offset and
   rectangular footprint.
9. Evidence-ranked warnings, such as control-rate shortfall, stale local plans,
   repeated angular saturation, localization jumps, and probable self returns.

An optional `--json PATH` output will contain the same summary in a stable schema
for comparing later runs. Event lists will remain bounded so reports do not grow
without limit.

## Analysis Semantics

- Bag record timestamps will drive cross-topic pairing; message header timestamps
  will also be checked for sensor timestamp lag where available.
- A command is paired with the nearest odom sample only within a configured maximum
  time separation. Unmatched samples are reported rather than silently compared.
- Direction changes ignore a configurable deadband to avoid counting noise around
  zero.
- `map -> odom` yaw differences are normalized across `+/-pi`.
- Rear-sector values near the persistent self-return distance are reported
  separately and excluded from wall-contact ranking by default.
- The analyzer will not label an event as a physical collision. It reports
  collision candidates and the evidence supporting them.

## Vehicle Configuration Synchronization

The development snapshot will be updated only where the read-only vehicle capture
showed a difference:

- `costmap_common_params.yaml` footprint becomes `+/-0.164` by `+/-0.122`.
- `teb_local_planner_params.yaml` uses the same vertices, `max_vel_y: 0.20`, and
  `acc_lim_y: 0.25`.

All other navigation YAML and launch files remain byte-for-byte unchanged unless
a fresh hash comparison proves a difference. The corresponding rows in
`manifest/source-map.csv` and entries in `manifest/files.sha256` will be updated
to preserve bundle integrity and identify the vehicle snapshot as the source of
the synchronized values.

The duplicate A*/DWA declarations and dead local inflation override will be
reported by a static validation test but will not be changed in this phase. They
are configuration-cleanup and tuning concerns that need a separate, reviewable
experiment.

## Tests

Implementation will follow red-green-refactor cycles.

Unit tests will cover:

- topic-rate and gap statistics;
- timestamp pairing and rejection of stale matches;
- angular saturation and sign-flip counting;
- `map -> odom` translation/yaw discontinuities, including yaw wraparound;
- rectangular body-clearance conversion from laser ranges;
- rear self-return classification;
- bounded event output and JSON schema;
- missing/empty topic handling.

Bundle tests will verify:

- synchronized footprint and TEB omnidirectional values;
- TEB and costmap footprint equality;
- detection of duplicate planner declarations and the ineffective inflation
  namespace without changing runtime behavior;
- updated manifest hashes;
- the existing `tests/validate_bundle.ps1` check.

The real bag regression will run read-only on the vehicle by streaming the local
script over SSH. Expected invariant metrics include approximately 819 scans,
1628 odom samples, 322 commands, repeated `1.2 rad/s` saturation, and material
`map -> odom` discontinuities. Exact floating-point values will use tolerances.

## Local Debugging Workflow

1. Run pure-Python unit tests on Windows.
2. Run bundle integrity/static configuration tests.
3. Stream the analyzer over SSH against the existing vehicle bag without writing
   to the vehicle.
4. Compare the report against the confirmed evidence above.
5. Only after analyzer and configuration synchronization are stable, design
   separate experiments for localization continuity, controller-loop frequency,
   footprint geometry, and omnidirectional smoothing.

## Safety and Acceptance Criteria

This phase is accepted when:

- all local tests pass;
- bundle validation passes with updated hashes;
- the analyzer runs against the LZ4 vehicle bag in the existing Noetic runtime;
- the generated report exposes the 5 Hz command rate, angular saturation, and
  localization discontinuities without relying on manual log interpretation;
- the development navigation values match the captured vehicle values;
- no file or runtime state on the vehicle has been modified.
