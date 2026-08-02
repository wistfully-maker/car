# Vehicle Navigation Configuration Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the development bundle's navigation footprint and TEB omnidirectional limits exactly match the read-only configuration captured from the vehicle.

**Architecture:** Extend the existing bundle validator with exact configuration assertions, make the smallest YAML edits required to match the vehicle, then refresh provenance and integrity hashes. Keep vehicle access read-only and leave analyzer enhancement for a separate plan.

**Tech Stack:** ROS1 YAML, PowerShell bundle validation, Git, SHA-256

---

### Task 1: Add vehicle-configuration regression checks

**Files:**
- Modify: `tests/validate_bundle.ps1`
- Modify: `manifest/files.sha256`
- Test: `tests/validate_bundle.ps1`

- [ ] **Step 1: Add exact failing assertions for the captured vehicle values**

Insert after the map YAML validation in `tests/validate_bundle.ps1`:

```powershell
$costmapCommonPath = Join-Path $bundleRoot 'source_snapshot\ucar_fast_nav\config\move_base\costmap_common_params.yaml'
$costmapCommonText = Get-Content -LiteralPath $costmapCommonPath -Raw -Encoding UTF8
$expectedFootprint = 'footprint: [[0.164, -0.122], [0.164, 0.122],[-0.164, 0.122], [-0.164, -0.122]]'
if ($costmapCommonText -notmatch "(?m)^$([regex]::Escape($expectedFootprint))\s*$") {
    $failures += 'costmap footprint does not match the captured vehicle footprint'
}

$tebPath = Join-Path $bundleRoot 'source_snapshot\ucar_fast_nav\config\move_base\teb_local_planner_params.yaml'
$tebText = Get-Content -LiteralPath $tebPath -Raw -Encoding UTF8
foreach ($expectedTebLine in @(
    '  acc_lim_y: 0.25',
    '  max_vel_y: 0.20',
    '    vertices: [[0.164, -0.122], [0.164, 0.122],[-0.164, 0.122], [-0.164, -0.122]]'
)) {
    if ($tebText -notmatch "(?m)^$([regex]::Escape($expectedTebLine))\s*$") {
        $failures += "TEB configuration missing captured vehicle value: $expectedTebLine"
    }
}
```

- [ ] **Step 2: Run the validator and verify RED**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\validate_bundle.ps1
```

Expected: exit code `1`, including failures for the costmap footprint, `acc_lim_y`, `max_vel_y`, and TEB vertices.

- [ ] **Step 3: Commit the failing regression checks**

Before committing, compute the new SHA-256 for `tests/validate_bundle.ps1` and
replace its matching entry in `manifest/files.sha256`. Re-run the validator and
confirm that only the four intended vehicle-configuration assertions fail.

```powershell
git add tests/validate_bundle.ps1 manifest/files.sha256
git commit -m "test: require captured vehicle navigation geometry"
```

### Task 2: Synchronize the two vehicle-derived YAML files

**Files:**
- Modify: `source_snapshot/ucar_fast_nav/config/move_base/costmap_common_params.yaml`
- Modify: `source_snapshot/ucar_fast_nav/config/move_base/teb_local_planner_params.yaml`
- Test: `tests/validate_bundle.ps1`

- [ ] **Step 1: Update the costmap footprint exactly**

Replace the footprint line with:

```yaml
footprint: [[0.164, -0.122], [0.164, 0.122],[-0.164, 0.122], [-0.164, -0.122]]
```

- [ ] **Step 2: Update the TEB lateral limits and footprint vertices exactly**

Use these vehicle-captured values:

```yaml
  acc_lim_y: 0.25
  max_vel_y: 0.20
  footprint_model:
    type: "point"
    vertices: [[0.164, -0.122], [0.164, 0.122],[-0.164, 0.122], [-0.164, -0.122]]
```

Do not change the point model, obstacle distances, inflation, forward/angular limits, or planner declarations in this synchronization task.

- [ ] **Step 3: Run the validator and verify the configuration assertions are GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\validate_bundle.ps1
```

Expected: the new vehicle-value failures disappear; the command still exits `1` only because manifest hashes have not yet been refreshed.

### Task 3: Refresh provenance and bundle integrity

**Files:**
- Modify: `manifest/source-map.csv`
- Modify: `manifest/files.sha256`
- Test: `tests/validate_bundle.ps1`

- [ ] **Step 1: Compute fresh SHA-256 values**

Run:

```powershell
$paths = @(
  'source_snapshot\ucar_fast_nav\config\move_base\costmap_common_params.yaml',
  'source_snapshot\ucar_fast_nav\config\move_base\teb_local_planner_params.yaml'
)
$paths | ForEach-Object {
  $hash = (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash.ToLowerInvariant()
  "$hash  $($_ -replace '\\','/')"
}
```

Expected: two lowercase 64-character SHA-256 values.

- [ ] **Step 2: Update source provenance rows**

For the two matching rows in `manifest/source-map.csv`:

- set `source_path` to the corresponding `/home/ucar/ucar_ws/src/ucar_fast_nav/config/move_base/<file>.yaml` path;
- set `role` to `runtime configuration synchronized from vehicle snapshot`;
- set both `source_sha256` and `bundle_sha256` to the fresh hash.

- [ ] **Step 3: Update bundle hash entries**

Replace the two matching hashes in `manifest/files.sha256` with the fresh values. Do not reorder unrelated entries.

- [ ] **Step 4: Run complete verification**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\validate_bundle.ps1
git diff --check
git status --short
```

Expected:

```text
PASS: navigation vendor bundle is internally consistent
```

`git diff --check` exits `0`. Status lists only the intended YAML and manifest changes plus the pre-existing untracked `tools/__pycache__/` directory; the validator is clean because its failing regression checks were committed in Task 1.

- [ ] **Step 5: Commit the synchronized snapshot**

```powershell
git add source_snapshot/ucar_fast_nav/config/move_base/costmap_common_params.yaml source_snapshot/ucar_fast_nav/config/move_base/teb_local_planner_params.yaml manifest/source-map.csv manifest/files.sha256
git commit -m "chore: sync vehicle navigation configuration"
```

The validator commit remains separate so the red-green history is reviewable.
