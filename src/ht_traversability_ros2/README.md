# HT 八方向可通过性 ROS2 节点

该包把过滤后的 elevation `GridMap` 输入新版 HT Scout student，在 CUDA 上进行
整图八方向推理，并发布可直接由 RViz GridMap 插件显示的结果。

## 接口

- 输入：`/elevation_mapping_node/elevation_map_filter`
- 输入层：`inpaint`
- 要求：`0.04 m/cell`，高程单位为米
- 输出：`/ht_traversability/traversability_8dir`
- 模型输入：`[1,1,H,W]`
- 模型输出：`[1,8,2,H-50,W-50]`

输出层：

- `ht_dir_0` ... `ht_dir_7`：八个原始方向的可通过概率。
- `ht_best_dir`：概率最高的原始通道编号。
- `ht_best_prob`：八方向最大概率。
- `ht_valid`：1 表示有足够有限值支持，0 表示未知。
- `ht_height`：原始高程，仅用于正确显示地形几何。

设置 `aggregate_only: true` 后，节点不再发布八个方向层，而只发布：

- `ht_traversable`：八个方向的概率全部大于等于
  `traversability_threshold` 时为 1，任一方向低于阈值时为 0，无效区域为 NaN。
- `ht_height`：仅供 RViz 按真实高程显示结果。

`aggregate_only` 和 `traversability_threshold` 支持运行时修改，例如：

```bash
ros2 param set /ht_traversability_node aggregate_only true
ros2 param set /ht_traversability_node traversability_threshold 0.5
```

八个通道尚未完成车辆坐标系标定，不应提前命名为前、左前等方向。规划器必须把
`ht_valid=0` 或概率为 NaN 的位置视为未知/不可通行。

## 构建和检查

```bash
cd /path/to/ele_scouts_kit
source /opt/ros/humble/setup.bash
./build_workspace.sh --packages-select ht_traversability_ros2
source install/local_setup.bash
source ./setup_runtime_env.sh
ros2 run ht_traversability_ros2 ht_preflight --device cuda
```

## 启动

只启动节点：

```bash
ros2 launch ht_traversability_ros2 ht_traversability.launch.py
```

同时启动预配置 RViz：

```bash
ros2 launch ht_traversability_ros2 ht_traversability.launch.py rviz:=true
```

检查输出：

```bash
ros2 topic hz /ht_traversability/traversability_8dir
ros2 topic echo /ht_traversability/traversability_8dir --once --field layers
```

RViz 预配置使用 `ht_height` 作为高度、`ht_best_prob` 作为颜色，并将颜色范围固定
为 0～1，避免每帧自动缩放造成颜色跳变。

节点默认在订阅前用 `250×250` 输入预热模型两次，提前完成 CUDA/cuDNN 的首次
形状初始化；可通过 `warmup_rows`、`warmup_cols` 和 `warmup_iterations` 调整。

## 静态墙体测试

`tools/run_static_wall_test.py` 可发布 1、3、…、49 cm 的隔离墙体和 5×5 总览图，
采集对应 HT 输出，并保存 NPZ、PNG、CSV、JSON 和 Markdown 报告。需要录包时先启动
HT 节点和 `ros2 bag record`，再用 `--expected-input-subscribers 2` 确保第一帧不会在
DDS 发现录包订阅者之前发出。

## 重要限制

模型感受野为 51×51。250×250 输入产生中心 200×200 输出，外围 25 格保持未知。
旧的 0.1 m/cell elevation map 不能直接输入该模型；需要从点云和真实 TF 重新
生成 0.04 m/cell elevation map。

该包只发布感知结果，不发送车辆控制指令。完成单向坡方向标定和真实地形验证前，
不得将其作为车辆唯一安全约束。
