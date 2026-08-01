# 另一组成功导航链独立整理 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将另一组实际运行的 lidar_loc + GlobalPlanner + TEB 导航链整理为不覆盖现有 `ucar_nav`、明天可直接部署的独立 ROS 包和可追溯归档。

**Architecture:** 新包 `ucar_fast_nav` 只拥有地图、move_base 参数、定位与导航组合 launch，不复制硬件驱动实现。`ucar_controller`、`ydlidar`、`jie_ware` 和 ROS planner 插件作为明确版本的外部依赖保存；首轮不启动尚未闭合的 dynamic_obstacle 链。

**Tech Stack:** ROS 1 Noetic、catkin、XML launch、YAML、GlobalPlanner、teb_local_planner、C++ lidar_loc、PowerShell 静态校验。

---

## 文件结构

- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/package.xml` — 新 ROS 包元数据。
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/CMakeLists.txt` — catkin 包声明及配置安装规则。
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/launch/navigation_full.launch` — 完整硬件与导航入口。
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/launch/navigation_stack.launch` — 仅地图、定位和 move_base 入口。
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/*.yaml` — 原样规划参数。
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/maps/002.{yaml,pgm}` — 原样比赛地图。
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/scripts/runtime_check.sh` — 部署后只读检查。
- Create: `navigation_vendor_bundle/vendor_dependencies/jie_ware/` — 定位依赖原始快照。
- Create: `navigation_vendor_bundle/vendor_dependencies/dynamic_obstacle/` — 未启用动态障碍链原始快照。
- Create: `navigation_vendor_bundle/manifest/files.sha256` — 归档文件哈希。
- Create: `navigation_vendor_bundle/manifest/source-map.csv` — 新路径到原路径的映射。
- Create: `navigation_vendor_bundle/docs/RUNTIME_CHAIN.md` — 节点、TF、topic 和参数来源。
- Create: `navigation_vendor_bundle/docs/DEPLOYMENT.md` — 明天人工部署、编译、启动和回滚步骤。
- Create: `navigation_vendor_bundle/docs/HANDOFF.md` — 当前结论、风险和实车验收记录入口。
- Create: `navigation_vendor_bundle/tests/validate_bundle.ps1` — Windows 本地静态检查。

### Task 1: 建立清单测试和目录骨架

**Files:**
- Create: `navigation_vendor_bundle/tests/validate_bundle.ps1`
- Create: `navigation_vendor_bundle/manifest/required-files.txt`

- [ ] **Step 1: 写失败的完整性检查**

脚本读取 `required-files.txt`，逐项用 `Test-Path -LiteralPath` 检查；缺失时打印 `MISSING: <relative path>` 并返回 1。

- [ ] **Step 2: 验证测试失败**

Run: `powershell -ExecutionPolicy Bypass -File navigation_vendor_bundle/tests/validate_bundle.ps1`

Expected: exit 1，并至少报告 `source_snapshot/ucar_fast_nav/package.xml` 缺失。

- [ ] **Step 3: 建立所需空目录**

使用 `New-Item -ItemType Directory` 创建设计中的目录；不创建空占位源码文件。

- [ ] **Step 4: 提交测试骨架**

```bash
git add navigation_vendor_bundle/tests navigation_vendor_bundle/manifest/required-files.txt
git commit -m "test: define navigation bundle completeness"
```

### Task 2: 提取地图与规划参数

**Files:**
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/maps/002.yaml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/maps/002.pgm`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/move_base_params.yaml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/costmap_common_params.yaml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/global_costmap_params.yaml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/local_costmap_params.yaml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/global_planner_params.yaml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config/move_base/teb_local_planner_params.yaml`

- [ ] **Step 1: 原样复制当前 launch 实际引用的文件**

来源固定为 `另一组小车/ucar/ucar_ws/src/ucar_nav/maps/002.*` 和 `launch/config/move_base/` 中六个被加载的 YAML，不使用同目录备份文件。

- [ ] **Step 2: 验证地图 YAML 图像引用**

Run: `Select-String -Path navigation_vendor_bundle/source_snapshot/ucar_fast_nav/maps/002.yaml -Pattern '^image:'`

Expected: 引用 `002.pgm`，不包含 `/home/ucar` 绝对路径。

- [ ] **Step 3: 验证配置关键项**

Run: `rg -n "GlobalPlanner|TebLocalPlannerROS|footprint|max_vel_y|inflation" navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config`

Expected: 找到全局规划、TEB 全向速度、footprint 和膨胀参数。

- [ ] **Step 4: 提交参数快照**

```bash
git add navigation_vendor_bundle/source_snapshot/ucar_fast_nav/maps navigation_vendor_bundle/source_snapshot/ucar_fast_nav/config
git commit -m "feat: snapshot proven navigation parameters"
```

### Task 3: 创建隔离的 ROS 包和 launch

**Files:**
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/package.xml`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/CMakeLists.txt`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/launch/navigation_stack.launch`
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/launch/navigation_full.launch`

- [ ] **Step 1: 写包元数据**

`package.xml` 声明 `roslaunch`、`map_server`、`move_base`、`global_planner`、`teb_local_planner`、`ucar_controller`、`ydlidar` 和 `jie_ware` 运行依赖。

- [ ] **Step 2: 写纯导航入口**

`navigation_stack.launch` 启动 `map_server`、`jie_ware/lidar_loc` 和 `move_base`，所有文件改用 `$(find ucar_fast_nav)`；保留可配置 `cmd_vel_topic`，删除 dynamic_obstacle include。

- [ ] **Step 3: 写完整入口**

`navigation_full.launch` include 现有 `ucar_controller/base_driver.launch`、`ydlidar/ydlidar.launch` 和新包的 `navigation_stack.launch`。提供 `start_base`、`start_lidar` 参数，默认均为 true，便于明天避免重复硬件节点。

- [ ] **Step 4: 检查不存在旧包路径**

Run: `rg -n "find ucar_nav|/home/ucar|dynamic_avoidance" navigation_vendor_bundle/source_snapshot/ucar_fast_nav`

Expected: 无输出。

- [ ] **Step 5: 提交独立包**

```bash
git add navigation_vendor_bundle/source_snapshot/ucar_fast_nav
git commit -m "feat: add isolated fast navigation package"
```

### Task 4: 保存外部依赖与来源映射

**Files:**
- Create: `navigation_vendor_bundle/vendor_dependencies/jie_ware/**`
- Create: `navigation_vendor_bundle/vendor_dependencies/dynamic_obstacle/**`
- Create: `navigation_vendor_bundle/manifest/source-map.csv`

- [ ] **Step 1: 原样复制 jie_ware**

复制 `CMakeLists.txt`、`package.xml`、`README.md`、`launch/` 和 `src/`，排除 build、devel 和日志。

- [ ] **Step 2: 原样复制 dynamic_obstacle 作为禁用参考**

保存源码、launch、msg 和包元数据，但在来源映射中标记 `runtime_enabled=false`。

- [ ] **Step 3: 写来源映射 CSV**

列固定为：

```csv
bundle_path,source_path,role,runtime_enabled
```

每个复制文件均有一行，路径使用正斜杠。

- [ ] **Step 4: 提交依赖快照**

```bash
git add navigation_vendor_bundle/vendor_dependencies navigation_vendor_bundle/manifest/source-map.csv
git commit -m "chore: archive navigation vendor dependencies"
```

### Task 5: 生成校验值和静态验证

**Files:**
- Create: `navigation_vendor_bundle/manifest/files.sha256`
- Modify: `navigation_vendor_bundle/tests/validate_bundle.ps1`

- [ ] **Step 1: 为所有归档文件生成 SHA256**

按相对路径排序，排除 `files.sha256` 自身，格式为 `<hash>  <relative path>`。

- [ ] **Step 2: 扩展验证脚本**

检查：必需文件齐全、地图图像存在、launch 不引用 `ucar_nav` 绝对资源、包名为 `ucar_fast_nav`、未启用 dynamic_obstacle、清单哈希与实际文件一致。

- [ ] **Step 3: 运行完整静态验证**

Run: `powershell -ExecutionPolicy Bypass -File navigation_vendor_bundle/tests/validate_bundle.ps1`

Expected: `PASS: navigation vendor bundle is internally consistent`，exit 0。

- [ ] **Step 4: 提交验证产物**

```bash
git add navigation_vendor_bundle/manifest navigation_vendor_bundle/tests
git commit -m "test: verify navigation bundle integrity"
```

### Task 6: 编写明日部署和交接文档

**Files:**
- Create: `navigation_vendor_bundle/source_snapshot/ucar_fast_nav/scripts/runtime_check.sh`
- Create: `navigation_vendor_bundle/docs/RUNTIME_CHAIN.md`
- Create: `navigation_vendor_bundle/docs/DEPLOYMENT.md`
- Create: `navigation_vendor_bundle/docs/HANDOFF.md`

- [ ] **Step 1: 写运行检查脚本**

脚本只读检查 `rospack find`、插件、`/scan`、`/odom`、`map->odom->base_link->laser_frame`、唯一 `/cmd_vel` 发布者和关键参数；任何失败返回非零，不发速度。

- [ ] **Step 2: 写部署文档**

列出明天的远端备份路径、上传位置、`catkin_make`、source、启动命令、停止命令和恢复原包命令。明确不覆盖 `/home/ucar/ucar_ws/src/ucar_nav`。

- [ ] **Step 3: 写运行链文档**

逐项记录节点所有者、输入输出 topic、TF 权限和实际参数文件，并说明 AMCL 备份与 lidar_loc 当前入口的区别。

- [ ] **Step 4: 写交接文档**

记录已确认事实、dynamic_obstacle 暂停原因、明天首轮验证顺序、成功标准和失败分层判断。

- [ ] **Step 5: 再次运行静态验证并提交**

Run: `powershell -ExecutionPolicy Bypass -File navigation_vendor_bundle/tests/validate_bundle.ps1`

Expected: PASS。

```bash
git add navigation_vendor_bundle
git commit -m "docs: add navigation deployment handoff"
```

### Task 7: 最终只读审计

**Files:**
- Verify: `navigation_vendor_bundle/**`

- [ ] **Step 1: 检查原始副本没有被修改**

对 `另一组小车/ucar/ucar_ws/src/ucar_nav` 和 `jie_ware` 重新生成哈希，与 source-map 记录的来源哈希比较。

- [ ] **Step 2: 检查 Git 变更范围**

Run: `git status --short`

Expected: 只有本计划、设计和 `navigation_vendor_bundle` 的预期变更；工作区原有无关未跟踪文件保持不变。

- [ ] **Step 3: 输出明天部署入口**

确认需上传的唯一路径为：

```text
navigation_vendor_bundle/source_snapshot/ucar_fast_nav
```

并确认若本组 `jie_ware` 不一致，使用：

```text
navigation_vendor_bundle/vendor_dependencies/jie_ware
```

- [ ] **Step 4: 最终提交**

```bash
git add navigation_vendor_bundle docs/superpowers
git commit -m "chore: finalize deployable navigation chain"
```
