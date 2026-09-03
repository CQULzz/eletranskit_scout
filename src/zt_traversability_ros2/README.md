# ZT 八向可通过性 ROS2 推理节点

这个包将 `ele_scouts_kit` 发布的过滤高程 `GridMap` 转换为 ZT student
模型张量，执行实时八向可通过性推理，再发布一个新的 `GridMap`。

```text
/rslidar_points
       ↓
ele_scouts_kit / elevation_mapping_cupy
       ↓  GridMap: inpaint
zt_traversability_node
       ↓  8-direction probability GridMap
/zt_traversability/traversability_8dir
```

## 已匹配的接口

- ROS2：Humble
- 输入话题：`/elevation_mapping_node/elevation_map_filter`
- 输入层：`inpaint`
- 输入地图：默认 250×250、0.04 m/格、米制高程
- 模型输入：`[1, 1, H, W] float32`
- 模型输出：`[1, 16, H-44, W-44]`
- 输出话题：`/zt_traversability/traversability_8dir`

250×250 输入会得到中心 206×206 的有效模型输出。外围 22 格没有完整的
45×45 感受野，保持 unknown。

## 输出层

- `zt_dir_0` ... `zt_dir_7`：八个原始方向通道的可通过概率，范围 0～1。
- `zt_best_dir`：八个通道中概率最高的通道编号 0～7。
- `zt_best_prob`：最高概率。
- `zt_valid`：1 表示该中心点周围有足够高程数据，0 表示未知。

注意：`zt_dir_0` ... `zt_dir_7` 暂时是模型原始通道，不应在方向标定前直接
命名为前、左前等车辆方向。可用 `direction_channel_order` 参数重排通道。

## 安装

直接克隆到 kit 工作区的 `src/`：

```bash
cd ~/ele_scouts_kit-main/src
git clone git@github.com:CQULzz/zt_trans.git zt_traversability_ros2
```

也可以下载 zip，再将整个目录复制到工作区：

```bash
cd ~/ele_scouts_kit-main
cp -r /path/to/zt_traversability_ros2 src/
```

确认 Jetson/AGX 已安装与 JetPack、CUDA 匹配的 PyTorch。不要在 Jetson 上盲目
安装普通 PC CUDA wheel。检查：

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
python3 -c "import scipy; print(scipy.__version__)"
```

构建并加载环境：

```bash
cd ~/ele_scouts_kit-main
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys python3-torch
colcon build --symlink-install --packages-select zt_traversability_ros2
source install/setup.bash
```

运行车载自检：

```bash
ros2 run zt_traversability_ros2 zt_preflight --device cuda
```

预期最后出现三行 `PASS`，并确认输入 `[1,1,45,45]` 得到
`[1,16,1,1]`。

## 运行

终端 1 启动原 kit：

```bash
cd ~/ele_scouts_kit-main
source install/setup.bash
ros2 launch ele_scout_kit real_car.launch.py
```

终端 2 启动 ZT：

```bash
cd ~/ele_scouts_kit-main
source install/setup.bash
ros2 launch zt_traversability_ros2 zt_traversability.launch.py
```

检查通讯：

```bash
ros2 topic hz /elevation_mapping_node/elevation_map_filter
ros2 topic hz /zt_traversability/traversability_8dir
ros2 topic echo /zt_traversability/traversability_8dir --once
```

默认 kit 高程图约 10 Hz，ZT 参数设为 5 Hz。确认 AGX 推理耗时足够低后，可将
`config/zt_real_car.yaml` 中的 `inference_fps` 调为 10.0。

## 同一个 launch 启动（可选）

先以两个终端方式验证。验证无误后，在
`src/ele_scout_kit/launch/real_car.launch.py` 中加入：

```python
Node(
    package="zt_traversability_ros2",
    executable="zt_traversability_node",
    name="zt_traversability_node",
    output="screen",
    parameters=[
        PathJoinSubstitution([
            FindPackageShare("zt_traversability_ros2"),
            "config",
            "zt_real_car.yaml",
        ])
    ],
)
```

并确保文件已有对应的 `Node`、`FindPackageShare` 和
`PathJoinSubstitution` import。之后启动 kit 的 real_car launch 就会同时启动 ZT。

## 必须进行的实车验证

1. 在平地、单向斜坡、台阶和沟槽上录制 rosbag，先离线回放。
2. 用方向明确的单向坡面标定八个通道与 `odom` 坐标轴的对应关系。
3. 标定后修改 `direction_channel_order`，再根据车辆 yaw 转成车体方向。
4. 规划器应把 `zt_valid=0` 或概率 NaN 的位置当作不可通行。
5. 初期保留 kit 原来的几何 `traversability` 作为额外安全约束。

本包只发布推理结果，不发送车辆速度指令，也没有修改现有规划器。给定权重是
旧版 45×45、五阶段 student，不是 ZT 源码中的六阶段 Scout 专用权重，未经
实车数据验证前不要单独用于车辆安全控制。

## 模型文件

仓库已经包含车端推理使用的 `models/student_8dir.pth`。teacher 权重仅用于训练
和知识蒸馏，不需要部署到车端。

student 权重 SHA256：

```text
4fb02eadc146555175faf3920eea39e0104ec292855036f83e455fc4af15fd06
```
