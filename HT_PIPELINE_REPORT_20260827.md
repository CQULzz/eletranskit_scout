# KIT 全局高程图 → 局部图 → HT 八方向通过性报告

日期：2026-08-27

## 修正说明

本报告对应 HT，不使用 ZT。运行日志中实际节点、权重和输出分别为：

- 节点：`ht_traversability_node`
- 权重：`student_best_epoch_weights.pth`
- SHA256：`0a79b865d012aa633fbb9913fc4031908dfe188ec9e161ef5a3fbb34a3f8c4d4`
- 输出：`/ht_traversability/global_local_8dir`
- 层：`ht_dir_0`～`ht_dir_7`、`ht_best_dir`、`ht_best_prob`、`ht_valid`、`ht_height`

第二次修正了全局窗口发布时多余的转置和双轴翻转。修正前生成的 HT GIF
不能用于判断全局框与局部图的对应关系，应以本文下方 `corrected...v2` 目录为准。

## 链路

`8 月点云 + 真实 TF/里程计 → 0.04 m 全局高程图 → SQLite 瓦片持久化 → 车辆中心 10 m × 10 m 局部裁剪 → HT Scout student → GIF`

HT 是八方向二分类网络，节点显示的是“可通过类别”的 softmax 概率，因此数值在 0～1 内连续变化；最终类别可按阈值转换为 0/1。

## 实测结果

- 输入 bag：`/mnt/nvme0n1/autoware/rosbag2_2026_08_20-17_21_56`
- 点云及 mapping ack：143/143 帧。
- 全局数据库 revision：143；瓦片：14；SQLite integrity：`ok`。
- 最后一帧局部图有效 cell：46,354。
- 最后一帧 HT 有效 cell：30,385。
- 最后一帧八方向最差概率中位数：0.5321。
- 最后一帧最佳方向概率中位数：0.8941。
- CUDA 预热后推理约 62～95 ms/帧。
- GIF：72 帧，1400 × 717，120 ms/帧。
- 71 个同时间戳可视化帧均通过逐 cell 全局/局部对齐检查：有限值掩码最低一致率 1.0，最大高程误差 0.0 m。

## 输出

- GIF：`/mnt/nvme0n1/autoware/global_ht_pipeline_corrected_20260827/rosbag2_2026_08_20-17_21_56_v2/recording/global_to_local_to_ht.gif`
- 最后一帧：`/mnt/nvme0n1/autoware/global_ht_pipeline_corrected_20260827/rosbag2_2026_08_20-17_21_56_v2/recording/gif_frames/frame_000071.png`
- 全局 SQLite：`/mnt/nvme0n1/autoware/global_ht_pipeline_corrected_20260827/rosbag2_2026_08_20-17_21_56_v2/global_tiles.sqlite3`
- 逐帧对齐统计：`/mnt/nvme0n1/autoware/global_ht_pipeline_corrected_20260827/rosbag2_2026_08_20-17_21_56_v2/recording/manifest.csv`

## 再次运行

```bash
cd /home/agx/ele_scout_kit_github_20260827
./run_august_bag_global_ht_gif.sh BAG目录 空输出目录 [ROS_DOMAIN_ID]
```

实车默认链路已改为 HT：

```bash
cd /home/agx/ele_scout_kit_github_20260827
./start_real_car.sh
```
