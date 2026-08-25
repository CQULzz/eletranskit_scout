# Third-party source snapshots

This repository vendors the source needed by the real-car application so it
does not depend on another local ROS workspace. Copyright and license files
remain in the corresponding source directories.

| Component | Snapshot | License | Bundled location |
|---|---|---|---|
| Fixposition driver lib, messages, ROS 2 node, FPSDK common and FPSDK ROS 2 | package version 8.0.0 / 1.0.0 | MIT | `src/vendor/fixposition` |
| rtcm_msgs | package version 1.1.6 | BSD-3-Clause | `src/vendor/fixposition/rtcm_msgs` |
| RoboSense rslidar_sdk | `78d2abb7f0c21f789a606d584475c631d5118dc3` | BSD-3-Clause | `src/vendor/robosense/rslidar_sdk` |
| RoboSense rs_driver | `4eeadace465023db65a63fc8ef680dfaaca5812c` | BSD-3-Clause | `src/vendor/robosense/rslidar_sdk/src/rs_driver` |
| RoboSense rslidar_msg | `fe8a95cb242bd294cc3d5e3422f2093fb49a56ee` | BSD-3-Clause | `src/vendor/robosense/rslidar_msg` |
| elevation_mapping_cupy and elevation_map_msgs | `8fd49565f622ae1481c0a22d6c8897441f7e2b7e` plus the current real-car fixes | MIT | `src/vendor/elevation` |
| ros2_numpy | `d5ebf723e1d2457d36fa89bf021b212e340c3c11` | MIT | `src/vendor/elevation/ros2_numpy` |
| semantic_sensor | bundled with the elevation mapping snapshot | MIT | `src/vendor/elevation/semantic_sensor` |

The top-level MIT license covers the original `ele_scout_kit` bringup code,
not third-party components under `src/vendor`.
