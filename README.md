# ele_scout_kit

面向当前实车的完整 ROS 2 Humble 工作区。仓库本身已经包含运行所需的程序
源码，不再依赖 `/home/agx/fp`、`/home/agx/rslidar_sdk`、
`/home/agx/elevation_map_cupy` 或 Autoware 工作区。

内置组件：

- Fixposition ROS 2 驱动及其全部必需库/消息包
- RoboSense `rslidar_sdk`、`rs_driver` 和 `rslidar_msg`
- 当前实车修改版 `elevation_mapping_cupy`、`elevation_map_msgs`、
  `ros2_numpy` 和 `semantic_sensor`
- 实车参数、三条静态 TF、正确的 ROS 2 RViz 配置
- 一键构建和一键启动脚本

第三方源码版本和许可证说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。ROS、CUDA、
CuPy 等系统运行环境仍需安装；这些属于操作系统/硬件依赖，不是其它工作区程序。

## 目录结构

```text
ele_scout_kit/
├── build_workspace.sh          # 构建整个独立工作区
├── start_real_car.sh           # 一键启动实车系统
└── src/
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
git clone git@github.com:CQULzz/ele_scout_kit.git
cd ~/ele_scout_kit
python3 -m pip install --user -r requirements-runtime.txt
./build_workspace.sh
```

脚本只会 source `/opt/ros/humble/setup.bash`，随后从本仓库 `src/` 构建全部
13 个 ROS 包。构建成功后可以确认各程序确实来自本仓库：

```bash
source /opt/ros/humble/setup.bash
source ~/ele_scout_kit/install/local_setup.bash
ros2 pkg prefix fixposition_driver_ros2
ros2 pkg prefix rslidar_sdk
ros2 pkg prefix elevation_mapping_cupy
```

三个输出都应位于 `~/ele_scout_kit/install/`。

## 一键启动并实时看图

启动前先关闭以前手工启动的 FP、雷达、建图节点和静态 TF 脚本，避免重复
发布：

```bash
cd ~/ele_scout_kit
./start_real_car.sh
```

默认同时启动：

1. Fixposition 驱动；
2. RSHELIOS 雷达驱动；
3. `FP_POI -> rslidar`、`FP_ENU0 -> map`、`map -> odom`；
4. 高程图节点（延迟 5 秒）；
5. RViz。

RViz 已设为 `Fixed Frame = odom`，默认显示：

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
src/ele_scout_kit/config/real_car.rviz
```

Fixposition 默认连接 `tcpcli://192.168.1.103:21000`。RSLidar 默认型号为
`RSHELIOS`，端口为 `3344/5566`，点云为 `/rslidar_points`，frame 为
`rslidar`。

### 无边界全局图

全局图默认位于 `odom`，分辨率为 0.04 m。它不是一张不断扩大的稠密 NumPy
数组，而是逻辑无边界的稀疏瓦片库：

- 每次点云成功融合后立即把局部后验更新到全局瓦片；
- 每块为 `128 × 128` 格，即 `5.12 × 5.12 m`；
- 只创建真正观测到的块，车辆远距离跳转不会分配中间空白区域；
- 内存最多缓存 256 块，单 elevation 层原始数据约 16 MiB；
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
才可改为 `true` 续建旧数据库。磁盘终究是有限的，应定期检查：

```bash
du -h ~/.ros/elevation_mapping_cupy/global_tiles.sqlite3*
```

清空内存和磁盘中的全局图：

```bash
ros2 service call /elevation_mapping_cupy/clear_global_map std_srvs/srv/Trigger '{}'
```

当前默认静态外参：

| 父坐标系 | 子坐标系 | 默认值 |
|---|---|---|
| `FP_POI` | `rslidar` | 全 0，仅供当前连通调试 |
| `FP_ENU0` | `map` | 全 0 |
| `map` | `odom` | 全 0 |

`FP_POI -> rslidar` 的零外参必须在获得实车标定值后替换，否则会产生高程
错位和重影。可临时通过 launch 参数覆盖：

```bash
./start_real_car.sh \
  poi_to_lidar_x:=0.0 poi_to_lidar_y:=0.0 poi_to_lidar_z:=0.0 \
  poi_to_lidar_roll:=0.0 poi_to_lidar_pitch:=0.0 poi_to_lidar_yaw:=0.0
```

## 运行检查

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /elevation_mapping_node/elevation_map_global
ros2 topic hz /elevation_mapping_node/elevation_map_global_tiles
ros2 run tf2_ros tf2_echo odom FP_POI
ros2 run tf2_ros tf2_echo odom rslidar
```

如果地图不显示，优先检查 `odom -> rslidar` 是否连通、FP 是否已输出
`FP_ECEF -> FP_ENU0` 和 `FP_ECEF -> FP_POI`，以及 RViz Fixed Frame 是否为
`odom`。

## 系统依赖

已在 NVIDIA AGX、Ubuntu 22.04、ROS 2 Humble、CUDA 12 环境验证。源码已经
内置，但新机器仍需具备 ROS/CUDA 和常规系统库，例如 Boost、Eigen、yaml-cpp、
libpcap、OpenCV、NumPy、SciPy、Shapely、CuPy。可先用 rosdep 检查常规 ROS
依赖：

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
python3 -c 'import cupy, cv2, numpy, scipy, shapely, ruamel.yaml, simple_parsing; print("Python dependencies OK")'
```
