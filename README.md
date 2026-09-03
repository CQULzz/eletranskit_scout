# eletranskit_scout

面向当前 Scout 实车的完整 ROS 2 Humble 工作区。仓库本身已经包含运行所需的程序
源码，不再依赖 `/home/agx/fp`、`/home/agx/rslidar_sdk`、
`/home/agx/elevation_map_cupy` 或 Autoware 工作区。

仓库名为 `eletranskit_scout`，其中的集成 ROS 包仍名为 `ele_scout_kit`。

内置组件：

- Fixposition ROS 2 驱动及其全部必需库/消息包
- RoboSense `rslidar_sdk`、`rs_driver` 和 `rslidar_msg`
- 当前实车修改版 `elevation_mapping_cupy`、`elevation_map_msgs`、
  `ros2_numpy` 和 `semantic_sensor`
- HT、ZT 八方向可通过性 ROS 2 节点和已验证的 student 模型权重
- 实车参数、三条静态 TF、正确的 ROS 2 RViz 配置
- 一键构建和一键启动脚本

HT、ZT 是仓库内的普通源码包，模型权重也保存在各自目录。它们不是软链接、
Git submodule，也不依赖 `/home/agx/ht_trans` 或其它本地工作区。

第三方源码版本和许可证说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。ROS、CUDA、
CuPy 等系统运行环境仍需安装；这些属于操作系统/硬件依赖，不是其它工作区程序。

## 目录结构

```text
eletranskit_scout/
├── build_workspace.sh          # 构建整个独立工作区
├── start_real_car.sh           # 一键启动实车系统
└── src/
    ├── ht_traversability_ros2/ # HT 节点、模型和测试
    ├── zt_traversability_ros2/ # ZT 节点、模型和测试
    ├── ele_scout_kit/          # launch、实车配置和 RViz
    └── vendor/
        ├── fixposition/        # FP 驱动、SDK、消息
        ├── robosense/          # RSLidar SDK、驱动、消息
        └── elevation/          # 高程图及其本地依赖
```

`build/`、`install/`、`log/` 是本仓库自己生成的构建产物，不需要任何外部
workspace 的 `install`。

## 首次构建

```bash
git clone git@github.com:CQULzz/eletranskit_scout.git
cd ~/eletranskit_scout
python3 -m pip install --user -r requirements-runtime.txt
./build_workspace.sh
```

脚本只会 source `/opt/ros/humble/setup.bash`，随后从本仓库 `src/` 构建全部
15 个 ROS 包。构建成功后可以确认各程序确实来自本仓库：

```bash
source /opt/ros/humble/setup.bash
source ~/eletranskit_scout/install/local_setup.bash
ros2 pkg prefix fixposition_driver_ros2
ros2 pkg prefix rslidar_sdk
ros2 pkg prefix elevation_mapping_cupy
ros2 pkg prefix ht_traversability_ros2
ros2 pkg prefix zt_traversability_ros2
```

四个输出都应位于当前 clone 目录的 `install/`。

## 一键启动并实时看图

启动前先关闭以前手工启动的 FP、雷达、建图节点和静态 TF 脚本，避免重复
发布：

```bash
cd ~/eletranskit_scout
./start_real_car.sh
```

默认同时启动：

1. Fixposition 驱动；
2. RSHELIOS 雷达驱动；
3. 组合 URDF 固定 TF 树、`FP_POI -> base_footprint`、`FP_ENU0 -> map`、`map -> odom`；
4. 高程图节点（延迟 5 秒）；
5. 仓库内 HT 八方向可通过性节点（延迟 7 秒）；
6. RViz。

RViz 已设为 `Fixed Frame = odom`。实时 HT 使用全局图中跟随车辆的 10 m 局部
裁剪 `/elevation_mapping_node/elevation_map_global_local`，默认发布保守综合结果
`/ht_traversability/global_local`：八个方向全部达到阈值才显示绿色，任一方向不满足
就显示红色。

RViz 默认显示：

- `/rslidar_points`
- `/elevation_mapping_node/elevation_map_global_tiles`：持续累积的稀疏全局高程瓦片

局部滚动图 `/elevation_mapping_node/elevation_map_raw` 和滤波图
`/elevation_mapping_node/elevation_map_filter` 仍保留，但在 Displays 中默认关闭。
`Global Detail Window` 也是默认关闭的可选高精度窗口。RViz 的 MarkerArray 会按
瓦片 ID 保留已走过区域；它不是把多帧局部图简单叠加。

当前 RSLidar SDK v1.5.19 停止时偶尔会打印一次 `std::system_error`。这是驱动
退出阶段的已知现象，不影响运行期间收点或 launch 停止其它节点。

不启动 RViz：

```bash
./start_real_car.sh start_rviz:=false
```

只做 launch 检查，不连接硬件：

```bash
./start_real_car.sh \
  start_fixposition:=false \
  start_rslidar:=false \
  publish_static_tf:=false \
  start_mapping:=false \
  start_rviz:=false
```

## 实车参数

所有默认参数都在本仓库内：

```text
src/ele_scout_kit/config/fixposition.yaml
src/ele_scout_kit/config/rslidar.yaml
src/ele_scout_kit/config/elevation_mapping_real_car.yaml
src/ele_scout_kit/config/ht_traversability_real_car.yaml
src/ele_scout_kit/config/real_car.rviz
src/ele_scout_kit/urdf/scout_sensor_rack.urdf.xacro
src/ht_traversability_ros2/models/student_best_epoch_weights.pth
```

Fixposition 默认连接 `tcpcli://192.168.1.103:21000`。RSLidar 默认型号为
`RSHELIOS`，端口为 `3344/5566`，点云为 `/rslidar_points`，frame 为
`rslidar`。

### 无边界全局图

全局图默认位于 `odom`，分辨率为 0.04 m。它不是一张不断扩大的稠密 NumPy
数组，而是逻辑无边界的稀疏瓦片库：

- 每次点云成功融合后，只把该帧真正更新的局部单元送入全局瓦片；
- 全局每格保存 `elevation`、`variance`、`observation_count` 和
  `last_update_revision`，使用方差加权更新，不再用新高度直接覆盖历史值；
- 新旧高度差门限取 3σ 和 0.30 m 中较小值，超过即拒绝写入；方差下限默认是
  0.0025 m²，避免重复相关观测产生虚假高置信度；
- 每块为 `128 × 128` 格，即 `5.12 × 5.12 m`；
- 只创建真正观测到的块，车辆远距离跳转不会分配中间空白区域；
- 内存最多缓存 256 块，四层瓦片原始数据约 64 MiB；
- 冷瓦片由后台线程压缩写入
  `~/.ros/elevation_mapping_cupy/global_tiles.sqlite3`，因此路线长度受磁盘容量而非
  固定地图边界限制。

ROS 的 `grid_map_msgs/GridMap` 本身必须是有限矩形，所以系统提供两个视图：

- `/elevation_mapping_node/elevation_map_global_tiles`：全局增量瓦片，供 RViz
  长路线总览；
- `/elevation_mapping_node/elevation_map_global`：默认 30 m × 30 m 的原分辨率
  细节窗口，不代表全局存储边界。

细节窗口默认中心是 `(0, 0)`。可以运行时移动，不会改动已保存数据：

```bash
ros2 param set /elevation_mapping_node global_map_view_center_x 40.0
ros2 param set /elevation_mapping_node global_map_view_center_y -15.0
```

也可以在实车 YAML 中将 `global_map_view_follow_robot` 改为 `true`，让细节窗口
跟随车辆。

`global_map_resume_from_disk` 默认是 `false`：每次启动建立一个新会话，避免
`odom` 重置后把两次路线错误拼接。只有确认 `odom/map` 的原点跨重启完全一致时，
才可改为 `true` 续建旧数据库。概率全局图使用 schema v2，旧版仅 elevation 的
schema v1 数据库不能续建，需要另存或删除后重新建图。磁盘终究是有限的，应定期检查：

```bash
du -h ~/.ros/elevation_mapping_cupy/global_tiles.sqlite3*
```

清空内存和磁盘中的全局图：

```bash
ros2 service call /elevation_mapping_cupy/clear_global_map std_srvs/srv/Trigger '{}'
```

当前默认静态外参来自组合后的 Scout + 传感器支架 URDF。这里按
`fixposition_link` 与驱动输出的 `FP_POI` 同位处理：

| 父坐标系 | 子坐标系 | 默认值 |
|---|---|---|
| `FP_POI` | `base_footprint` | `xyz=(-0.2005158179, 0, -0.65899)`, `rpy=(0,0,0)` |
| `base_footprint` | `base_link` | `xyz=(0, 0, 0.23479)`, `rpy=(0,0,0)` |
| `base_link` | `sensor_kit_base_link` | `xyz=(0.1725, 0, 0.1162)`, `rpy=(0,0,0)`（网格接触高度） |
| `sensor_kit_base_link` | `rs_helios_link` | `xyz=(0, 0, 0.434)`, `rpy=(0,0,0)` |
| `rs_helios_link` | `rslidar` | 全 0（驱动 frame 别名） |
| `FP_ENU0` | `map` | 全 0 |
| `map` | `odom` | 全 0 |

因此等效的 `FP_POI -> rslidar` 是
`xyz=(-0.0280158179, 0, 0.126)`、`rpy=(0,0,0)`。如果实车的 Fixposition
内部 POI 不在设备网格原点，必须用实测 POI 外参修正下面的桥接值，否则仍会产生
高程错位和重影。可通过 launch 参数覆盖：

```bash
./start_real_car.sh \
  poi_to_base_footprint_x:=-0.2005158179 \
  poi_to_base_footprint_y:=0.0 \
  poi_to_base_footprint_z:=-0.65899 \
  poi_to_base_footprint_roll:=0.0 \
  poi_to_base_footprint_pitch:=0.0 \
  poi_to_base_footprint_yaw:=0.0
```

## 运行检查

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /elevation_mapping_node/elevation_map_global
ros2 topic hz /elevation_mapping_node/elevation_map_global_tiles
ros2 topic hz /elevation_mapping_node/elevation_map_global_local
ros2 topic echo /elevation_mapping_node/drift_compensation_status
ros2 topic hz /ht_traversability/global_local
ros2 run tf2_ros tf2_echo odom FP_POI
ros2 run tf2_ros tf2_echo odom rslidar
```

如果地图不显示，优先检查 `odom -> rslidar` 是否连通、FP 是否已输出
`FP_ECEF -> FP_ENU0` 和 `FP_ECEF -> FP_POI`，以及 RViz Fixed Frame 是否为
`odom`。

实车配置已启用基于相邻点云 TF 运动门控的公共 Z 轴漂移补偿。状态数组字段、
安全门限、32 线 LiDAR 的 A/B 验收方案及回退方法见
[Z_AXIS_DRIFT_COMPENSATION_PLAN_20260903.md](Z_AXIS_DRIFT_COMPENSATION_PLAN_20260903.md)。

## 系统依赖

已在 NVIDIA AGX、Ubuntu 22.04、ROS 2 Humble、CUDA 12 环境验证。所有应用源码
和 HT 模型已经内置，但新机器仍需具备 ROS/CUDA 和常规系统库，例如 Boost、
Eigen、yaml-cpp、libpcap、OpenCV、NumPy、SciPy、Shapely、CuPy，以及与该机器
JetPack/CUDA 匹配的 NVIDIA PyTorch。可先用 rosdep 检查常规 ROS 依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r --rosdistro humble \
  --skip-keys "ament_python cupy-cuda12x simple-parsing python3-torch python3-torchvision"
python3 -m pip install --user -r requirements-runtime.txt
```

`requirements-runtime.txt` 包含可安全通用安装的 Python 依赖；CuPy 必须另行
选择与该机器 CUDA/JetPack 匹配的版本，不建议由脚本盲目覆盖。本机当前使用
`cupy-cuda12x 13.6.0`。可用下面的命令检查关键模块：

```bash
python3 -c 'import cupy, cv2, numpy, scipy, shapely, ruamel.yaml, simple_parsing, torch; print("Python dependencies OK", torch.cuda.is_available())'
source install/local_setup.bash
source ./setup_runtime_env.sh
ros2 run ht_traversability_ros2 ht_preflight --device cuda
```

`setup_runtime_env.sh` 和 `start_real_car.sh` 会从当前 Python 环境自动发现 NVIDIA PyTorch 附带的 CUPTI，
不会假设用户名、Python 小版本或固定的 `/home/agx` 路径。若 CUDA/PyTorch 尚未
安装，脚本会在启动硬件前明确报告缺少的模块。
