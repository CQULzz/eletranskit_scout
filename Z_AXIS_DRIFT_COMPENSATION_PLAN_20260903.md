# Ele Scout Z 轴漂移补偿启用计划

日期：2026-09-03

## 实施状态

- [x] ROS2 点云回调接入真实 TF 运动量，不再固定传 0；
- [x] 实车配置显式启用 Z 轴漂移补偿；
- [x] 关闭 mapper 内置可通过性时，维持漂移筛选所需的中性置信层；
- [x] 增加逐帧补偿状态话题和 CPU 单元测试；
- [x] 使用三个 bag 的 TF 轨迹完成门限预检；
- [ ] 在 CuPy/GPU 目标机完成三个 bag 的开/关 A/B 出图与指标验收。

当前本机没有 CuPy/GPU，且 ROS 环境缺少 `grid_map_msgs` 系统包，因此 GPU
内核运行和完整 ROS 构建属于目标机验收项；纯 Python 打包、语法、配置、运动
门控单测及真实 TF 门限预检已完成。

## 1. 目标与边界

目标是在 `elevation_mapping_cupy` 的点云建图路径中真正启用现有 Z 轴漂移补偿，降低定位公共 Z 偏移导致的历史高程与当前扫描错层。

本功能只修正整帧公共 Z 偏移，不负责：

- LiDAR 一圈扫描期间的运动畸变（deskew）；
- roll/pitch、XY 配准或外参误差；
- 对录制的 elevation 类话题进行读取或复用；
- 修改原始 TF 或 Fixposition 数据。

## 2. 当前问题

`enable_drift_compensation` 已设为 `true`，但 ROS2 点云回调把 `position_noise` 和 `orientation_noise` 固定传为 `0`。补偿内核要求两者至少一个超过门限，因此正常建图路径不会触发。

上游原版同样保留了这段未接通路径。本项目采用 TF 相邻帧运动量补齐门控信号，不通过伪造常量绕过安全条件。

## 3. 实施方案

### 3.1 运动门控

- 按点云订阅源分别保存上一帧 `map <- lidar` 位姿；
- 平移量采用相邻帧欧氏距离，单位 m；
- 旋转量采用 SO(3) 测地角，单位 rad；
- 分别使用现有 `position_lowpass_alpha` 和 `orientation_lowpass_alpha` 做 EMA；
- 首帧输出 0，不允许首帧补偿；
- 默认门限保持 `0.01 m` 与 `0.01 rad`。

### 3.2 高度补偿

保持 kit 原有安全策略：

- 可信历史格的匹配数必须大于 100；
- 可信格要求有效、低方差且 traversability 大于 0.9；实车关闭 mapper
  内置可通过性网络时，有效格的该内部置信层维持中性值 1，避免车辆移动一个
  局部图长度后新格始终为 0、导致补偿失效；这不会替代或修改下游 HT 输出；
- 平均残差 `mean(point_z - map_z)` 的绝对值必须小于 0.1 m；
- 每帧仅应用残差的 10%，即 `correction = 0.1 * mean_error`；
- 补偿历史局部图后再融合本帧点云。

### 3.3 可观测性

新增 `/elevation_mapping_node/drift_compensation_status`，每个成功处理的点云发布一次 `Float32MultiArray`：

1. `enabled`
2. `motion_gate_open`
3. `position_motion_m`
4. `orientation_motion_rad`
5. `inlier_count`
6. `mean_error_m`（未估计时为 NaN）
7. `applied_correction_m`
8. `cumulative_correction_m`
9. `applied`

## 4. 32 线 LiDAR 参数起点

第一轮保持保守参数：

| 参数 | 初值 | 说明 |
|---|---:|---|
| `min_height_drift_cnt` | 100 | 32 线每帧点数足够，仍要求足量可信重叠 |
| `max_drift` | 0.10 m | 拒绝大跳变 |
| `drift_compensation_alpha` | 0.10 | 避免一次性拉动整图 |
| `position_noise_thresh` | 0.01 m | 移动门控 |
| `orientation_noise_thresh` | 0.01 rad | 转动门控，约 0.57° |
| `position_lowpass_alpha` | 0.20 | 平移运动量 EMA |
| `orientation_lowpass_alpha` | 0.20 | 旋转运动量 EMA |

不在没有 A/B 数据的情况下先行放宽残差、可信格或最大漂移门限。

## 5. 验证与验收

### 5.1 代码级

- 首帧、静止、平移、旋转、多传感器独立状态测试通过；
- 配置检查确认真实车辆配置显式开启；
- 原 elevation mapping 回归测试不新增失败。

### 5.2 Bag A/B

对三个原始 bag 分别生成“补偿关闭”和“补偿开启”结果。两组只允许读取 `/rslidar_points`、`/tf`、`/tf_static`，禁止读取所有 elevation 相关话题。

重点比较：

- 同一路段去程/回程的公共高度偏差；
- 邻格高度差 P50/P95；
- 坡度 P95；
- 8 方向可通过性中位数差异；
- 每帧 inlier 数、平均残差及实际补偿量；
- 是否出现长时间单向累计或错误吸附到路沿/植被。

通过标准：公共 Z 偏差及去回程差异下降，平坦路面粗糙度不恶化；不得出现单帧超过 1 cm 的实际修正（当前参数理论上小于 1 cm），不得出现持续无界累计。

## 6. 风险与回退

- 斜坡上存在 XY 偏差时，高度残差可能混入水平配准误差；
- roll/pitch 误差会产生空间变化残差，公共 Z 补偿无法消除；
- 动态物体和非地面回波可能污染普通均值；
- 当前点云没有逐点时间字段，Z 补偿不能代替 deskew。

若 A/B 结果恶化，将 `enable_drift_compensation` 设为 `false` 即可回退，不改变原始数据和 TF。后续增强应优先采用地面 ROI 的稳健统计（中位数/截尾均值），并单独解决逐点 deskew。

## 7. 交付阶段

1. 接通 TF 运动门控并显式启用真实车辆配置；
2. 增加状态输出与自动测试；
3. 在可用 CuPy/GPU 环境运行三个 bag 的严格 A/B；
4. 根据 A/B 指标决定是否调整 32 线专用参数；
5. Z 补偿稳定后，再单独实施点云 deskew。
