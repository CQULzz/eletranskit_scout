# KIT 实时全局高程图 → 局部图 → ZT 8 向通过性报告

日期：2026-08-27

## 结论

链路已在最新 GitHub KIT 的独立工作区中实现并跑通：

`/rslidar_points + TF/里程计 → elevation_mapping_cupy → SQLite 稀疏全局图 → 10 m × 10 m 局部裁剪 → ZT 8 向通过性 → GIF`

原有工作区 `/home/agx/ele_scout_kit` 未被覆盖；实现位于：

`/home/agx/ele_scout_kit_github_20260827`

## GitHub 最新版与当前本地版差异

- GitHub 最新提交：`e0c4e4d4cfe34c26b671cf821e7bb5fa645c978d`（`Add ele scout kit`）。
- 当前本地提交：`9e00dd347d38519545f62775e005cb8037b8d8cb`。
- 二者共同祖先：`149fc26fb22c80254b2592d3212ed00c1bfc5329`。
- 已提交源码内容基本相同；GitHub 版把 11 个启动/节点脚本的权限从 `755` 改成了 `644`，会导致 ROS 直接启动 Python 节点时报“executable not found”。新工作区已恢复正确执行权限。
- 两个 Fixposition 二进制测试样本有少量字节差异，不影响运行链路。
- 原工作区另有 5 个未提交的车体三维 crop 修改。本实现没有覆盖原目录，而是把正确的 X/Y/Z crop 迁移到了新工作区。

## 实现内容

- 全局高程图继续采用 128 × 128 cell 的稀疏瓦片，分辨率 0.04 m，内存缓存并异步刷新到 SQLite。
- 新增从持久全局瓦片中独立裁剪局部图的发布器：
  - 全局图：`/elevation_mapping_node/elevation_map_global`
  - 10 m 局部图：`/elevation_mapping_node/elevation_map_global_local`
  - ZT 输出：`/zt_traversability/global_local_8dir`
- ZT 直接订阅全局图裁剪结果，不再使用 mapper 自带的滚动局部图作为输入。
- ZT 输出包含 `zt_dir_0` 至 `zt_dir_7`、`zt_valid`、`zt_best_prob`、`zt_best_dir` 和 `zt_height`。
- 实车启动默认开启 ZT，并设置 CUDA/CUPTI 运行库路径。
- RViz 配置新增固定 0～1 色域的 ZT 显示，避免自动色域每帧跳动。
- 离线 runner 对每一帧点云等待 mapping receipt/ack，保证慢速处理时不丢帧。
- 车体自滤波参数为 X `[-1.0, 0.2]` m、Y `[-0.7, 0.7]` m、Z `[-0.9, 0.2]` m。

## 8 月 rosbag 实测

输入：`/mnt/nvme0n1/autoware/rosbag2_2026_08_20-17_21_56`

- bag 时长：16.55 s；点云：143 帧。
- mapping 收到并确认：143/143 帧。
- SQLite revision：143；level-0 瓦片：14；`PRAGMA integrity_check = ok`。
- 全局有效索引范围：X `[-142, 287]`、Y `[-125, 158]`，分辨率 0.04 m。
- 最后一帧 10 m 局部图有效 cell：46,354。
- 最后一帧 ZT 有效 cell：32,250；8 向最差概率中位数 0.451，最佳方向概率中位数 0.740。
- ZT 使用 CUDA；除首次模型预热 518 ms 外，记录到的单帧推理大约 74～112 ms。
- GIF：72 帧，1400 × 385，120 ms/帧，约 7.3 MB。

开头第一张采样帧的 ZT 是全 NaN，因为此时有效高程覆盖不足；随着全局图积累，有效区域正常增长。这是模型有效掩码行为，不是输出故障。

## 输出位置

- 推荐查看的新版 GIF：`/mnt/nvme0n1/autoware/global_zt_pipeline_20260827/rosbag2_2026_08_20-17_21_56_v2/recording/global_to_local_to_zt.gif`
- 新版 GIF 左侧大图显示累计全局图、车辆轨迹、当前位置和橙色 10 × 10 m 裁剪框；箭头连接右上局部图，右下显示 ZT 最差/最佳方向概率。
- 新版 GIF 为 72 帧、1400 × 717、150 ms/帧。
- 全局图 SQLite：`/mnt/nvme0n1/autoware/global_zt_pipeline_20260827/rosbag2_2026_08_20-17_21_56/global_tiles.sqlite3`
- 每帧 PNG：`/mnt/nvme0n1/autoware/global_zt_pipeline_20260827/rosbag2_2026_08_20-17_21_56/recording/gif_frames`
- 同步帧统计：`/mnt/nvme0n1/autoware/global_zt_pipeline_20260827/rosbag2_2026_08_20-17_21_56/recording/manifest.csv`
- 汇总：`/mnt/nvme0n1/autoware/global_zt_pipeline_20260827/rosbag2_2026_08_20-17_21_56/summary.json`

## 使用方法

离线处理另一个 8 月 rosbag（输出目录必须为空）：

```bash
cd /home/agx/ele_scout_kit_github_20260827
./run_august_bag_global_zt_gif.sh BAG目录 输出目录 [ROS_DOMAIN_ID]
```

实车完整链路：

```bash
cd /home/agx/ele_scout_kit_github_20260827
./start_real_car.sh
```

实车全局数据库默认写到：

`/mnt/nvme0n1/autoware/live_global_elevation/global_tiles.sqlite3`
