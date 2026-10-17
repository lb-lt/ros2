# Gazebo 无人车激光雷达自主探索建图

## 概述

本模块在 Gazebo 11 + ROS Noetic 中搭建**无人车（差速驱动地面车）** 的物理仿真环境，
使用车载**二维激光雷达**完成感知，并实现了一个**自主探索建图**控制器，
配合栅格 SLAM 在线生成占用栅格地图（occupancy grid）。

模块聚焦「无人车」这一地面机器人形态，覆盖课程任务书中要求的三个环节：

| 环节 | 实现 | 证据 |
|------|------|------|
| 物理仿真与运动控制 | TurtleBot3 waffle_pi 差速车 + `cmd_vel` 速度控制 | [Gazebo 运行截图](#1-gazebo-仿真环境) |
| 传感器感知与控制 | 360° 激光雷达 `/scan` → 扇区势场 → `cmd_vel` | [感知-控制相关性图](#3-激光雷达感知与控制闭环) |
| SLAM 建图与导航 | 在线栅格 SLAM + 自主探索，覆盖率提升 3.3 倍 | [建图覆盖率对比](#4-slam-建图结果) |

![Gazebo 中的无人车仿真世界](img/ground/gazebo_tb3_car_explore/gazebo_world_launched.png)

## 环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 20.04.6 LTS (focal) |
| ROS | Noetic |
| Gazebo | 11.13.0 |
| 机器人模型 | TurtleBot3 waffle_pi |
| Python | 3.8 + numpy |

本模块在课程提供的虚拟机镜像 `ros_noetic_humble_gazebov11_linux_win_v1` 中验证通过，
依赖该镜像内已安装的 `turtlebot3_gazebo`、`turtlebot3_description`、`gazebo_ros`
以及 `hutb_robotics_sim`（提供 `task3_world.world` 场景与 `lidar_slam` 节点）。

## 安装与运行

### 1. 放入工作空间并编译

```bash
mkdir -p ~/car_ws/src
cp -r gazebo_tb3_car_explore ~/car_ws/src/
cd ~/car_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

若使用课程镜像，`hutb_robotics_sim` 已经在 `~/openhutb_car_ws` 中编译好，
可以直接把本包放进 `~/openhutb_car_ws/src/` 后重新 `catkin_make`：

```bash
cp -r gazebo_tb3_car_explore ~/openhutb_car_ws/src/
cd ~/openhutb_car_ws && catkin_make
```

### 2. 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/openhutb_car_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

roslaunch gazebo_tb3_car_explore explore_map.launch
```

启动后 Gazebo 打开 `task3_world` 场景，无人车从 `(-0.5, -0.5)` 出发，
`explore_mapping` 节点驱动其自主漫游，RViz 中实时显示 `/map`、`/scan` 与机器人模型。

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `linear_speed` | 0.18 m/s | 巡航线速度 |
| `turn_speed` | 0.9 rad/s | 逃逸时原地转向角速度 |
| `safe_distance` | 0.55 m | 触发逃逸的正前方距离阈值 |
| `use_rviz` | true | 是否启动 RViz |

```bash
# 无界面运行（服务器 / 远程调试）
roslaunch gazebo_tb3_car_explore explore_map.launch use_rviz:=false

# 提高巡航速度
roslaunch gazebo_tb3_car_explore explore_map.launch linear_speed:=0.24
```

### 3. 一键入口

如只需复现实验并采集数据，可直接运行包内入口脚本：

```bash
python3 $(rospack find gazebo_tb3_car_explore)/main.py
```

`main.py` 会依次：检查依赖 → 启动 `explore_map.launch` → 等待建图 →
统计覆盖率 → 保存地图与轨迹到 `data/` 目录。

`--duration` 控制建图积累时长（默认 120 s），覆盖率随运行时间增长。
实测对照（同一场景）：

| `--duration` | 已知空闲 | 已知占据 | 覆盖率 |
|--------------|---------|---------|--------|
| 75 s | 7,360 | 726 | 1.26% |
| 120 s | 14,552 | 1,139 | 2.45% |

## 算法说明

### 1. Gazebo 仿真环境

`task3_world.world` 是一个内嵌障碍物的室内场景（圆柱立柱、矩形墙体、斜置隔板），
**不依赖联网下载 `model://` 资源**，因此可以离线立即启动。

无人车模型采用 `turtlebot3_waffle_pi`：

- 差速驱动：`gazebo_ros_diff_drive` 插件，订阅 `/cmd_vel`，发布 `/odom` 与 `odom→base_footprint` TF
- 激光雷达：360 线（`angle_min=0, angle_max=2π`，步长 `0.0175 rad`），发布 `/scan`
- 相机与 IMU：`/camera/rgb/image_raw`、`/imu`

实测 Gazebo 实时因子 `Real-Time Factor ≈ 0.99~1.00`，`FPS ≈ 52`，
说明仿真在虚拟机内可达到实时运行。

车载激光雷达在 RViz 中的实时可视化（红色为 `/scan` 原始点，`Global Status: Ok`，
固定坐标系 `odom`）：

![激光雷达 RViz 可视化](img/ground/gazebo_tb3_car_explore/laser_scan_rviz.png)

### 2. 激光扇区势场逃逸控制器

`explore_mapping.py` 每个控制周期把一帧 `/scan` 划分为五个扇区：

```
        前 (-0.35, 0.35)
   左前 (0.35,1.05)   右前 (-1.05,-0.35)
 左 (1.05,1.75)        右 (-1.75,-1.05)
```

控制逻辑：

1. **无数据守卫**：若整帧五个扇区都没有有效回波，判定传感器异常，**立即停车**。
   注意「没有回波」不等于「前方空旷」，这条守卫是第 4 节修复的缺陷所在。
2. **逃逸模式**：正前方距离 `< safe_distance` 时进入逃逸，朝左右空间更大的一侧
   以 `turn_speed` 原地旋转，直到正前方距离 `>= escape_distance`（滞回，避免抖动）。
3. **巡航模式**：正前方通畅时，线速度按剩余距离线性缩放，角速度按左右前扇区
   距离差做居中修正：

   $$v = v_{max} \cdot \mathrm{clip}\!\left(\frac{d_{front} - d_{safe}}{0.8}, 0, 1\right)$$

   $$\omega = \mathrm{clip}\!\left(0.9\,(d_{front\_left} - d_{front\_right}), -0.6, 0.6\right)$$

与课程 `nn_lidar_avoidance` 的纯 MLP 相比，本控制器带有**显式滞回与守卫**，
不会陷入「正对障碍时转向力矩不足」的局部极小。

### 3. 激光雷达感知与控制闭环

下图是任务二 `nn_lidar_avoidance` 神经网络避障的实测数据（40 个采样点，每 0.3 s 一次），
横轴为采样序号，红线为网络输出的角速度，蓝线为同一时刻激光最近距离：

![激光最近距离与角速度输出相关性](img/ground/gazebo_tb3_car_explore/nn_lidar_control.png)

可以清楚看到闭环成立：第 6~8 个采样点距离从 0.23 m 快速下降到 0.15 m 时，
角速度同步上升到峰值 `+0.40 rad/s`（向左逃离）；越过障碍后距离回升，
角速度反向到 `-0.61 rad/s` 做反向修正。这种**符号反转**正是「绕行通过」而非
「原地打转」的特征。

### 4. SLAM 建图结果

`lidar_slam` 以纯 NumPy 实现 log-odds 贝叶斯栅格更新，发布 `/map`
（800×800，分辨率 0.05 m/格，原点 `(-20, -20)`）。

![SLAM 占用栅格地图](img/ground/gazebo_tb3_car_explore/slam_occupancy_map.png)

左半部为 RViz 中的占用栅格地图：圆柱障碍呈现为黑色圆环，矩形墙体与斜置隔板清晰可见，
灰色为已探索空闲区域，深色为未知区域；右半部为 Gazebo 中实际的无人车与障碍物，
两者几何一致。

建图覆盖率对比如下（同一 `task3_world` 场景，同一时长）：

![建图覆盖率对比](img/ground/gazebo_tb3_car_explore/mapping_coverage.png)

| 方案 | 已知空闲 | 已知占据 | 覆盖率 |
|------|---------|---------|--------|
| 课程 `nn_lidar_avoidance` | 4,293 | 498 | 0.75% |
| 本模块探索器（修复前） | 6,482 | 713 | 1.12% |
| **本模块探索器（修复后）** | **14,552** | **1,139** | **2.45%** |

覆盖率相对课程 MLP 提升 **3.3 倍**。

### 5. 里程计轨迹

![无人车里程计轨迹](img/ground/gazebo_tb3_car_explore/odom_trajectory.png)

## 运动控制实测

在 Gazebo 中通过 `/cmd_vel` 发布速度指令，读取 `/odom` 验证无人车真实响应：

| 阶段 | 指令 | 实测位置 | 实测线速度 |
|------|------|---------|-----------|
| 初始 | — | (-1.9999, -0.4997) | ~0 |
| 前进 3 s | v = 0.15 m/s | (-1.3578, -0.4981) | **0.1502 m/s** |
| 左转 3 s | ω = 0.6 rad/s | (-0.8750, -0.5101) | 0 |
| 停止 | 0 | (-0.8605, -0.5084) | **0.0002 m/s** |

前进阶段位移 0.642 m、线速度 0.1502 m/s 与指令 0.15 m/s 吻合，
停止后速度归零，说明**速度控制链路真实有效**。

激光雷达在同一场景下的统计：`360` 线中 `354` 条有效回波，
测距范围 `0.370 ~ 3.483 m`，角度分辨率 `0.0175 rad`。

## 数据文件

实测原始数据保存在 `src/gazebo_tb3_car_explore/data/`：

| 文件 | 内容 |
|------|------|
| `nn_lidar_pairs.csv` | 任务二 40 组「激光最近距离 / 线速度 / 角速度」采样 |
| `odom_trajectory.csv` | 任务三 `/odom` 轨迹（360 个位姿点） |
| `map_scan.pgm` | 建图得到的 800×800 占用栅格（P5 二进制 PGM） |
| `lidar_mlp.npz` | 训练好的激光雷达 MLP 权重（49 KB） |

## 已知限制

1. **探索覆盖率受场景与时长限制**：本文数据为约 2 分钟建图的结果，
   覆盖率 2.45% 针对整幅 40 m×40 m 地图而言；若延长运行时间可继续提升。
2. **`match_enabled=false`**：建图阶段使用纯里程计积分，
   仿真里程计精度高，但换到真实机器人需要开启 scan-to-map 匹配。
3. **局部极小依然可能发生**：显式守卫解决了「无回波误判」与「正对障碍」两类问题，
   但在宽阔空腔内仍可能来回巡游，未实现前沿（frontier）主动探索。

## 参考资料

* [TurtleBot3 仿真文档](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
* [gazebo_ros 插件](http://gazebosim.org/tutorials?tut=ros_gzplugins)
* [Occupancy Grid Map 与 log-odds 更新](https://wiki.ros.org/map_server)
* [OpenHUTB/ros2 无人车文档](https://openhutb.github.io/ros2/)
