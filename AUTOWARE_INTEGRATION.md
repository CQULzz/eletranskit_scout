# Autoware Scout Integration Snapshot

Date: 2026-10-23. Source branch:
`feature/autoware-ht-integration-2026-10-23`.
Candidate tag: `ele-scout-ht-v0.1.0-rc.2`.

## Repository Boundary

The independent publication target for this repository is
`git@github.com:multimeters/ele_scouts_kit.git`.
The original `CQULzz/ele_scouts_kit` remote and its history are retained.
Autoware is maintained separately in
`git@github.com:Unmanned-Systems-Lab/autoware.scout.git`.
Autoware's `repositories/scout-external.repos` pins the matching ELE commit.
Do not copy ELE's complete driver/control launch into an active Autoware stack.

## Included Source

- Existing local HT/global-elevation changes: explicit directional probability,
  binary directional cost and aggregate output switches; finite-support masks;
  bounded height filtering; observation-aware variance-weighted global tiles;
  asynchronous snapshots and diagnostic/replay helpers.
- The existing optional ZT adapter and checkpoint are preserved as part of the
  current source snapshot. Autoware's new integration selects HT, not ZT.
- Existing source reports are retained as historical documentation. Their
  past performance measurements are not a validation of a new deployment.
- Build/install/log, recordings, map databases and runtime screenshot output
  are excluded. The checked-in model and source licenses remain unchanged.

## Autoware Consumer Contract

Autoware uses this checkout's `ht_traversability_ros2` package and the Scout
checkpoint `models/student_best_epoch_weights.pth`, SHA256:
`0a79b865d012aa633fbb9913fc4031908dfe188ec9e161ef5a3fbb34a3f8c4d4`.

The companion launcher enables `ht_dir_0..7` probabilities and `ht_valid`,
disables the all-directions aggregate, and publishes to `/scout/ht/raw`.
It reads `elevation` from
`/elevation_mapping_node/elevation_map_global_local` at 0.04 m/cell.
`ht_cost_dir_k` is binary (0 free, 1 blocked), whereas `ht_dir_k` is a
traversability score (higher is better). They are not interchangeable.
The Autoware adapter applies the model front-reference shift exactly once.
Direction mapping and operational thresholds still require field validation.

The companion Autoware feature currently performs observation-only fusion.
It does not modify planner inputs or provide a validated terrain-driving mode.

## Validation

On 2026-10-23, 47 tests passed for HT, ZT and the global tile accumulator:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source setup_runtime_env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q --import-mode=importlib \
  src/ht_traversability_ros2/test src/zt_traversability_ros2/test \
  src/vendor/elevation/elevation_mapping_cupy/elevation_mapping_cupy/tests/test_global_map.py
```

Disabling unrelated auto-loaded ROS pytest plugins avoids duplicate `test_core`
collection conflicts; this is not the complete elevation/CUDA test suite.
The HT CUDA preflight and an isolated synthetic elevation -> actual HT model ->
Autoware fusion test also passed. No real vehicle driving validation is claimed.
