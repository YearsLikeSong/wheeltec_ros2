# V550 麦轮：手动目标点，边建图边导航

本工作区使用 ROS 2 Humble。默认链路为：原底盘/雷达驱动 → EKF → SLAM Toolbox 在线建图 → Nav2 → `/cmd_vel`。启动后等待你指定目标，不会自动选择探索目标，不需要提前提供地图，也不需要 AMCL 的初始位姿操作。

## 一条启动命令

```bash
bash /home/wheeltec/wheeltec_ros2/tools/v550.sh
```

等 RViz 中出现地图、激光且 Navigation 2 显示 active 后，选择工具栏 **Nav2 Goal**，在目标位置按下鼠标并拖动箭头指定朝向。Nav2 持续重新规划，SLAM 同时更新地图。第一轮先在已观测的空闲区域给一个近距离目标，确认雷达方向、车体位置和行驶方向一致。

这里“未知区域”表示启动前没有地图。默认全局规划器允许经过地图内的未知栅格；目标必须在当前 `/map` 的范围内。地图外的远点不能直接规划，沿新显露的区域逐步给点即可。二维激光只能避开其扫描平面看见的障碍，不具有台阶/悬崖检测能力。

没有桌面显示或使用远程 RViz：

```bash
bash /home/wheeltec/wheeltec_ros2/tools/v550.sh use_rviz:=false
```

在小车本机另一个终端可发送明确坐标（单位米，yaw 为弧度，坐标系 `map`）：

```bash
source /home/wheeltec/wheeltec_ros2/tools/v550_env.sh
ros2 run v550_navigation send_goal 1.0 0.5 --yaw 0.0
```

示例客户端会等待导航结果，失败返回非零退出码；超时或 Ctrl-C 会请求取消自己的目标。RViz 中使用 **Cancel** 取消图形界面发出的目标。结束整套程序前先 Cancel，再在启动终端按 Ctrl-C；如需有序卸载 Nav2 插件，可先执行 `ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 4}"`，然后 Ctrl-C。不要同时运行键盘、语音移动、巡航或其他向 `/cmd_vel` 发指令的节点。

## 保存地图与编译

建图运行期间另开终端：

```bash
bash /home/wheeltec/wheeltec_ros2/tools/save_v550_map.sh my_map
```

保存为 `maps/my_map.yaml` 和地图图像，已有同名地图时拒绝覆盖。不传名称时自动使用时间戳。该命令保存占据栅格，不是用于恢复 SLAM 的序列化位姿图。

本次修改的五个包已在本机编译。以后修改这些包后：

```bash
cd /home/wheeltec/wheeltec_ros2
bash tools/build_v550.sh
```

此脚本复用当前已安装的 Nav2 和外设依赖。新机器从源码完整部署时，先安装 ROS 2 Humble 及 `rosdep` 解析出的依赖，再运行 `colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF`。原厂家驱动中的串口权限、SDK/库及设备规则仍需按目标机器配置；本次没有改动这些驱动。

## 接入自己的算法

| 你要替换的部分 | 接口或启动方式 |
| --- | --- |
| 目标选择、任务调度、探索策略 | 调用 `/navigate_to_pose`，类型 `nav2_msgs/action/NavigateToPose`；最小可运行示例是 `src/v550_navigation/scripts/send_goal.py` |
| 全局规划/局部控制 | 实现 Nav2 `nav2_core::GlobalPlanner` / `nav2_core::Controller` 插件，在自己的 YAML 中替换 `planner_server` / `controller_server` 插件配置，通过 `params_file:=/绝对路径/my_nav2.yaml` 加载 |
| 自己完成规划和控制 | 启动时加 `start_nav2:=false`，自己的节点发布 `geometry_msgs/msg/Twist` 到 `/cmd_vel`；麦轮使用 `linear.x`、`linear.y`、`angular.z`，自行负责避障、限速及失联停车 |
| 自己完成 SLAM | 加 `start_slam:=false`，自己的 SLAM 提供 `/map` 和 `map → odom_combined` TF |
| 底盘/传感器已在别处启动 | 加 `bringup_hardware:=false`，避免重复占用串口或重复发布 TF |

例如保留建图、使用自己的运动算法：

```bash
bash tools/v550.sh start_nav2:=false
```

自建包放到 `src/`，例如：

```bash
cd /home/wheeltec/wheeltec_ros2/src
ros2 pkg create my_algorithm --build-type ament_python \
  --dependencies rclpy nav_msgs sensor_msgs geometry_msgs nav2_msgs tf2_ros
cd ..
colcon build --packages-select my_algorithm
source install/setup.bash
```

| 输入/输出 | ROS 接口 |
| --- | --- |
| 实时地图 | `/map`：`nav_msgs/msg/OccupancyGrid`，订阅建议使用 reliable + transient local QoS |
| 雷达 | `/scan` 为原始数据；`/scan_slam` 为建图使用的固定 1440 点扫描，均为 `sensor_msgs/msg/LaserScan`，使用 sensor data QoS；`start_slam:=false` 时适配节点也关闭 |
| 原始里程计 / 融合里程计 | `/odom` / `/odom_combined`：`nav_msgs/msg/Odometry` |
| IMU | `/imu/data_raw`；Madgwick 输出 `/imu/data` |
| 坐标变换 | `map → odom_combined → base_footprint → base_link → laser/camera_link`；IMU 为 `base_footprint → gyro_link` |
| 速度输出 | `/cmd_vel`：`geometry_msgs/msg/Twist` |

参数入口：`src/v550_navigation/config/nav2_v550.yaml` 和 `slam_v550.yaml`。默认 DWB 已启用正负 Y 横移，横移速度阈值为 0.001 m/s；最大平移速度为 0.20 m/s，角速度为 0.5 rad/s。车体碰撞外形按所附 V550 CAD 范围向外取整为 0.32 × 0.28 m，再加 0.01 m padding；加装外设超出该范围时需扩大 footprint。

## 驱动、模型与清理记录

- 全部雷达、深度/USB 相机、GPS/RTK、IMU、麦克风/语音、人体识别外设、手柄、键盘、充电和视频服务源码保留，并与清理前 SHA256 对比。激光跟随包内失去依赖的 ArUco 演示入口也已清理；语音入口依赖的 `simple_follower_ros2`、`nav2_waypoint_cycle`、`ollama_ros_chat`/消息包也保留。
- 本机 Nav2 在工作区内编译安装，系统 `/opt/ros/humble` 没有 Nav2 bringup，因此保留整套 Nav2 源码与插件，方便以后开发和比较自己的算法。
- 删除 14 个与当前任务无关的包：`aruco`、`aruco_ros`、`aruco_msgs`、`ultralytics_ros2`、`wheeltec_multi`、`wheeltec_path_follow`、`wheeltec_robot_kcf`、`wheeltec_robot_rrt`、`wheeltec_rrt_msg`、`wheeltec_robot_rtab`、`orb_slam2_ros`、`openslam_gmapping`、`slam_gmapping`、`wheeltec_cartographer`。同步清除了各自 `build/`、`install/`，并删除其他车型的 URDF、网格和导航参数。语音导航硬编码的 `param_mini_mec.yaml` 文件名由 CMake 安装为当前 V550 参数的兼容副本，保留驱动源码不变。
- V550 URDF 本身已有 `laser`、`camera_link` 的固定关节，因此取消旧 launch 对这两个 frame 的重复发布，底盘高度使用原代码标注的 0.02027 m。根据实车轮子驱动时的雷达/里程计对比，已将 `laser_joint` 的 yaw 从 CAD 中的 -π 修正为 0，安装位置保持原值。
- 雷达入口仍使用原配置 `Lsm10_m10p_uart`、`/dev/wheeltec_lidar`；底盘仍使用 `/dev/wheeltec_controller`、115200；IMU 默认用底盘数据。它们是当前代码选择，不能据此确认实际硬件型号。需要原外置 Yesense IMU 时加 `use_wheeltec_imu:=true`。相机默认不参与二维导航，需要时加 `start_camera:=true`，沿用原 Astra Pro 入口。

备份位于 `backups/v550_before_cleanup/original_sources.tar.gz`，删除清单位于同目录 `removed.json`，原外设文件校验值为 `peripheral_sha256.json`。`backups/COLCON_IGNORE` 防止备份被 colcon 编译。恢复时先解压到独立目录查看，不直接覆盖当前工作区：

```bash
mkdir -p /tmp/v550_original
tar -xzf backups/v550_before_cleanup/original_sources.tar.gz -C /tmp/v550_original
```

## 验证边界

五个修改包编译通过；合成数据测试成功建图并完成 (0.7, 0.5) m 目标，位置误差 0.148 m（配置容差 0.15 m），观测到 0.113 m/s 横移速度。目标超时取消、地图保存测试也通过。导航运动闭环测试在软件环境中进行，未驱动实车运动。随后已采集用户手动驱动实车的直走和转弯数据，并用隔离回放验证雷达朝向及建图持续更新，见下方动态建图修正记录。`tools/test_v550_navigation.py` 使用独立 ROS 域和合成的激光/里程计，启动真实 SLAM Toolbox 与 Nav2 并下发测试目标；它不验证实际串口、雷达朝向、轮速标定、地面打滑或相机。

```bash
source install/local_setup.bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/v550_ros_test \
  python3 tools/test_v550_navigation.py
```

实现方式参考 [Nav2 官方的边建图边导航说明](https://docs.nav2.org/lyrical/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/)，具体启动接口和插件名以本机 Humble 源码为准。


## 点云旋转修正与重复启动保护

2026-09-26 的实车静态采样确认：车轮里程计为零时，原 IMU 积分航向仍漂移，旧 EKF 把它作为高可信度绝对航向融合，15 秒内车体 TF 变化约 0.74°。已作两项修正：

- EKF 不融合板载 IMU 的积分航向，保留角速度与麦轮 X/Y 速度融合；正常转弯的角速度门限改为 5 sigma。
- 同一有效串口帧的 IMU 与里程计使用同一个采样时间戳，在串口读完后计算采样间隔，避免把同一次采集当成先后两次测量。

修正后，在禁用 Nav2 控制的单实例实车静态采样中：180 帧激光的相邻帧角度配准偏移均为 0（配准分辨率 0.5°）；15 秒车体 TF 航向变化约 0.00049°；`map → odom_combined` 航向变化为 0；地图正常以 1 Hz 发布。没有发送速度或导航目标。原始记录为 `backups/v550_heading_fix/stationary_before.json` 和 `stationary_after.json`。

“第二次启动后快速旋转”与重复发布 TF/重复使用串口的现象相符，但当时的双实例状态没有复现，不能把它当作已证实的唯一原因。默认启动入口现已加入互斥锁和已有底盘/雷达进程检测，第二次启动会给出提示并退出。锁由进程退出自动释放，不需要删锁文件。`bringup_hardware:=false` 用于有意复用外部传感器，它跳过硬件锁；这种组合仍要自行保证只有一套 EKF、SLAM 和导航控制节点。

重新启动时先退出旧终端中的建图/导航，再只执行一次：

```bash
bash /home/wheeltec/wheeltec_ros2/tools/v550.sh
```

RViz 的 **Fixed Frame 保持 `map`**。在地图坐标下，墙面点云应大致固定，车体模型随实际小车移动；若使用 `base_footprint` 或 `laser` 作为固定坐标，环境点云随小车运动而变化属于该显示方式的正常结果。

额外验证：`tools/test_v550_startup_guard.py` 验证并发启动拒绝、正常退出后重新启动和旧驱动占用检测；`tools/test_v550_ekf.py` 在隔离 ROS 域验证静止漂移抑制、真实转弯角速度和横移估计。已有外设源码完整性检查仍通过。仅针对底盘数据时间戳修改了 `turn_on_wheeltec_robot/src/wheeltec_robot.cpp`，未改雷达驱动；之后根据动态采样修正了雷达坐标朝向，见下节。

此处对测量量和协方差的处理参考 [robot_localization Humble 配置说明](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst)。修改前底盘源码、EKF 配置和启动入口保存在 `backups/v550_heading_fix/`。


## 点云随车移动：动态建图修正

用户手动驱动实车的记录 `log/v550_motion_recording.json.gz` 表明：原始 `/odom`、融合 `/odom_combined` 和车体 TF 均随运动变化，原始里程计最终位移约 0.688 m、融合位移约 0.698 m。故这次问题并非里程计未启动。

已定位并修正两项问题：

- 原雷达每帧点数在 1653～1697 之间变化。SLAM Toolbox/Karto 缓存首帧几何，随后报 `LaserRangeScan contains ... range readings, expected ...` 并拒收扫描。新增 `v550_scan_normalizer`，把 `/scan` 按固定 0.25° 网格转换为 `/scan_slam`（1440 点），每格取最近有效回波，保留原时间戳、frame 和盲区，不跨缺失区插值。原雷达驱动、原始 `/scan` 和 Nav2 的原始激光输入保持原样。
- `laser_joint` 原有的 180° 旋转与实际扫描坐标不符。18 个有效平移片段中，按 yaw=0 计算的帧间雷达/里程计位移平均差约 0.0112 m，按原 yaw=π 计算约 0.1231 m，已据此修正为 0。

建图的扫描等待队列同时设置为 10，以容纳 12 Hz 雷达等待 10 Hz 里程计 TF 的时间。原模型与配置备份于 `backups/v550_motion_fix/`。

同一段约 27 秒的真实行驶数据隔离回放：原配置只有 1 个建图位姿、1 版地图；修正后有 11 个建图位姿、10 版地图，地图校正 TF 随运动更新，点数不匹配报错消失。结果分别在 `log/v550_replay_before.json` 和 `log/v550_replay_corrected.json`。原始采集与回放过程中，诊断程序没有发送车辆运动命令。回放证明建图恢复更新；实车自动导航仍需在现场检验地面打滑和轮速标定。

`tools/test_v550_scan.py` 验证变长扫描、障碍物方向、近障保留、盲区和时间戳。只读诊断现在还报告地图内容版本数，避免仅凭 `/map` 的发布频率判断建图正常：

```bash
source /home/wheeltec/wheeltec_ros2/tools/v550_env.sh
python3 /home/wheeltec/wheeltec_ros2/tools/diagnose_v550_tf.py --seconds 30 \
  --output /home/wheeltec/wheeltec_ros2/log/v550_tf_diagnostic.json \
  --record /home/wheeltec/wheeltec_ros2/log/v550_drive_recording.json.gz
```

运行以上诊断前仅启动一套 `tools/v550.sh`，再由操作者手动低速移动。程序只订阅数据，不发布速度。

安装后的实车静止检查确认 `/scan_slam` 约 12 Hz、每帧固定 1440 点，`base_link → laser` yaw 为 0；检查结束后所有诊断硬件进程已退出。完整导航回归再次通过，目标误差 0.139 m、最大横移速度 0.113 m/s，并完成取消、地图保存及有序退出。1120 个外设文件的完整性校验保持通过。

保留的厂家雷达驱动在收到 SIGINT 退出时返回过 `-6`，修复前后均观察到；该退出问题未在本次修改外设源码处理。采集期间扫描持续输出，退出后已确认串口驱动进程没有残留。它与此次扫描点数不一致导致的运行中拒收是两件事。


## 局部规划轻抖、停滞的参数调整

针对最近实车日志中的 `Failed to make progress` 和 `No valid trajectories`，在 `nav2_v550.yaml` 中调整 DWB：`ObstacleFootprint.scale` 从 1.0 改为 0.02，`PathAlign.scale` 从 32.0 改为 8.0，`min_speed_xy/min_speed_theta` 从 0/0 改为 0.05 m/s / 0.10 rad/s。后两项在此 Humble 实现中联合过滤“平移和转向同时过小”的候选，仍允许纯转向，并不保证每个带转向的指令都有 0.05 m/s 平移。碰撞判定、车体外形、膨胀层和最大速度保持原配置。未启用的 BaseObstacle/GoalAlign 评分参数已移除，避免误调无效项。

软件对照测试设置前墙 X=1.25 m、目标 (0.9, 0) m，并假设平移速度低于 0.04 m/s 时模拟车不动；这个门限是测试条件，不是实测电机参数。旧参数 55 秒超时，目标误差 0.287 m；新参数到达目标，误差 0.135 m，测试中车体与前墙最小间距 0.281 m。另一项横移加 90° 终点转向测试也通过，位置误差 0.132 m；取消、地图保存及有序退出正常。实车实际门限和地面效果仍需复测。

参数已重新安装，并同步厂家语音导航的兼容参数副本。重启 `bash tools/v550.sh` 生效。正常的每秒重规划保留；`debug_trajectory_details` 已开启，若仍出现无可用轨迹，日志会指出具体拒绝原因。调整前参数保存在 `backups/v550_controller_tuning/nav2_before.yaml`，软件对照结果保存在 `log/v550_navigation_tuning_before.json`、`v550_navigation_tuning_after.json` 和 `v550_navigation_tuning_lateral_turn.json`。


## ROS 网络数据混入与隔离

2026-09-27 检查时，本机底盘程序全部退出后，默认 ROS 域仍能收到网络中的另一套雷达、底盘、EKF 和 SLAM。仅本机通信的对照探测没有这些发布者。本机重新启动后，`/scan`、`/odom`、`/odom_combined` 均出现两个不同 GID 的发布者，融合位姿在原点和约一米外交错，车体 TF 航向跨度约 84°。当时 RViz 还曾收到指向 `mini_mec` 的模型。证据保存在 `backups/v550_network_fix/` 和 `log/v550_mixed_network_diagnostic.json`；尚不能确定外部程序属于哪台设备。

`tools/v550_env.sh` 现在默认 `ROS_LOCALHOST_ONLY=1`，完整启动、地图保存和编译辅助脚本都引用此环境。适用于在小车本机运行全部 ROS 节点的方式；通过远程桌面访问小车仍可使用。文件修改不会改变已运行节点的通信模式，必须退出旧的建图、导航、键盘和录包程序，再重启。也可明确指定：

```bash
ROS_LOCALHOST_ONLY=1 bash /home/wheeltec/wheeltec_ros2/tools/v550.sh
```

小车上的其他 ROS 终端，包括键盘、算法节点、诊断和录包，在启动节点前使用相同环境：

```bash
source /home/wheeltec/wheeltec_ros2/tools/v550_env.sh
```

如果该终端以前明确设置过 `ROS_LOCALHOST_ONLY=0`，先执行 `export ROS_LOCALHOST_ONLY=1`。辅助脚本尊重用户显式设置，不会把它覆盖掉。`ros2 topic list` 等命令可能使用旧 ROS daemon 的发现缓存；通信模式切换后可以先执行 `ros2 daemon stop` 清理该缓存，或使用支持的 `--no-daemon` 查询选项。

若确实需要另一台电脑本地运行 ROS/RViz，而不是远程桌面，则在小车及那台电脑的所有相关终端都显式设置 `ROS_LOCALHOST_ONLY=0`，并设置同一个、与其他小车不同的 `ROS_DOMAIN_ID`，然后重启各节点。例如双方约定域 55（应先确认没有其他车使用）：

```bash
export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=0
```

不要在还运行其他车辆的默认域 0 中共享同名 `/scan`、`/odom`、`/tf`、`/cmd_vel`。只读诊断工具现在会列出这些单车话题的多发布者问题。
