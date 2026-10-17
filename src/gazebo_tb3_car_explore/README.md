# gazebo_tb3_car_explore

Gazebo 无人车（TurtleBot3 waffle_pi）激光雷达自主探索建图模块 —— ROS Noetic + Gazebo 11。

湖南工商大学《智能机器人》课程「地面移动无人车」选题的模块实现。

## 功能

* Gazebo 物理仿真：差速驱动无人车 + 360° 二维激光雷达 + 相机 + IMU
* 自主探索控制：激光扇区 + 势场逃逸（带无数据守卫与滞回）
* 在线栅格 SLAM：log-odds 贝叶斯占用栅格建图，发布 `/map`
* 一键入口 `main.py` 与数据采集脚本 `record_run.py`

## 快速开始

```bash
# 1. 编译
cp -r gazebo_tb3_car_explore ~/openhutb_car_ws/src/
cd ~/openhutb_car_ws && catkin_make

# 2. 运行
source /opt/ros/noetic/setup.bash
source ~/openhutb_car_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch gazebo_tb3_car_explore explore_map.launch

# 或者一键入口
python3 $(rospack find gazebo_tb3_car_explore)/main.py
```

## 运行环境

| 项目 | 版本 |
|------|------|
| Ubuntu | 20.04.6 LTS |
| ROS | Noetic |
| Gazebo | 11.13.0 |
| Python | 3.8（numpy） |
| 机器人 | TurtleBot3 waffle_pi |

依赖课程镜像中已有的 `turtlebot3_gazebo`、`turtlebot3_description`、`gazebo_ros`
与 `hutb_robotics_sim`（提供 `task3_world.world` 与 `lidar_slam`）。

## 目录结构

```
gazebo_tb3_car_explore/
├── CMakeLists.txt
├── package.xml
├── main.py                      # 一键入口
├── launch/
│   └── explore_map.launch       # 仿真 + 探索 + SLAM + RViz
├── scripts/
│   ├── explore_mapping.py       # 激光扇区势场逃逸控制器
│   └── record_run.py            # 运行数据采集
├── rviz/
│   └── explore.rviz             # 地图/激光/模型可视化配置
└── data/                        # 实测数据（CSV / PGM / 模型权重）
```

## 运行结果

* Gazebo 实时因子 ≈ 1.00，FPS ≈ 52（虚拟机内实时运行）
* 运动控制：指令 0.15 m/s → 实测 0.1502 m/s；停止后实测 0.0002 m/s
* 激光雷达：360 线中 354 条有效回波，测距 0.370~3.483 m
* 建图覆盖率 2.45%，为课程 MLP 基线（0.75%）的 **3.3 倍**

详细说明与效果图见 [`docs/ground/gazebo_tb3_car_explore.md`](../../docs/ground/gazebo_tb3_car_explore.md)。

## 许可

MIT
