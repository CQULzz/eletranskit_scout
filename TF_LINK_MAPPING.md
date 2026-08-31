# Scout 组合模型 Link / TF 对应说明

## 基本规则

- URDF 中每个 `<link name="...">` 都会成为一个同名 TF frame。
- `<joint>` 的 `parent`、`child` 和 `origin xyz/rpy` 定义两个 link frame 之间的 TF。
- 本模型的关节全部是 `fixed`，所以 `robot_state_publisher` 将它们发布到 `/tf_static`。
- `<visual><origin>` 只改变网格相对 link frame 的显示位置，不会新增 TF。本模型的网格原点均为零。

## 完整主链

```text
Fixposition 驱动/launch                         URDF robot_state_publisher

FP_ECEF ──> FP_POI ──> base_footprint ──> base_link ──> sensor_kit_base_link
                                                    ├── sensor_frame_link
                                                    ├── fixposition_link
                                                    ├── gnss_left_link
                                                    ├── gnss_right_link
                                                    ├── imu_tm431_link
                                                    ├── rs_m1_link
                                                    ├── zed2i_link
                                                    └── rs_helios_link ──> rslidar
```

`FP_POI -> base_footprint` 由 KIT launch 发布；从 `base_footprint` 开始的模型树由
`robot_state_publisher` 根据 URDF 发布。

## Link / TF / 实物对应

| URDF link / TF frame | 实际含义 | 父 frame | xyz (m) |
|---|---|---|---|
| `base_footprint` | 车辆在地面的投影参考点，无网格 | `FP_POI` | `-0.2005158179, 0, -0.65899` |
| `base_link` | Scout 底盘主体坐标系 | `base_footprint` | `0, 0, 0.23479` |
| `front_right_wheel_link` | 右前轮 | `base_link` | `0.249, -0.29153, -0.0702` |
| `front_left_wheel_link` | 左前轮 | `base_link` | `0.249, 0.29153, -0.0702` |
| `rear_left_wheel_link` | 左后轮 | `base_link` | `-0.249, 0.29153, -0.0702` |
| `rear_right_wheel_link` | 右后轮 | `base_link` | `-0.249, -0.29153, -0.0702` |
| `sensor_kit_base_link` | 整个传感器支架的标定原点，无网格 | `base_link` | `0.1725, 0, 0.1162` |
| `sensor_frame_link` | 支架实体网格 | `sensor_kit_base_link` | `0, 0, 0` |
| `fixposition_link` | Fixposition 外壳/模型参考点 | `sensor_kit_base_link` | `0.0280158179, 0, 0.308` |
| `gnss_left_link` | 左 GNSS 天线 | `sensor_kit_base_link` | `0, 0.31, 0.313` |
| `gnss_right_link` | 右 GNSS 天线 | `sensor_kit_base_link` | `0, -0.31, 0.313` |
| `imu_tm431_link` | TM431 IMU | `sensor_kit_base_link` | `0, 0, 0.4195` |
| `rs_m1_link` | RS M1 | `sensor_kit_base_link` | `0.0299201190, 0, 0.187` |
| `zed2i_link` | ZED2i 相机 | `sensor_kit_base_link` | `0.0999943369, 0, 0.2481252405` |
| `rs_helios_link` | RS Helios 机械/模型坐标系 | `sensor_kit_base_link` | `0, 0, 0.434` |
| `rslidar` | RSLidar 驱动点云消息使用的 frame 名 | `rs_helios_link` | `0, 0, 0` |

除右侧两个车轮按上游网格设置了 `roll=pi` 外，表中各 TF 的 RPY 均为零。

## 不属于 URDF 的定位/地图 Frame

| frame | 来源 | 用途 |
|---|---|---|
| `FP_ECEF` | Fixposition 驱动 | 地心地固全局参考系 |
| `FP_ENU0` | Fixposition 驱动 | 本地东-北-天坐标原点 |
| `FP_POI` | Fixposition 驱动 | 定位输出的车辆兴趣点（POI） |
| `map` | KIT 静态桥 | 建图全局坐标系 |
| `odom` | KIT 静态桥 | 高程图与 RViz 当前使用的里程计坐标系 |

当前模型假设 `FP_POI` 与 `fixposition_link` 同位。因此整条链计算后：

```text
FP_POI -> fixposition_link = (0, 0, 0), RPY=(0,0,0)
FP_POI -> rslidar          = (-0.0280158179, 0, 0.126), RPY=(0,0,0)
```

如果 Fixposition 设备内部配置的 POI 与外壳网格原点不同，应修改
`poi_to_base_footprint_*` launch 参数，而不是单独移动 `rslidar`。
