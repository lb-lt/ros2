# OpenHUTB ROS/ROS2 文档与示例 —— 无人车专题（lb2）

本仓库是 `OpenHUTB/ros2` 的一个分支（`lb2`），聚焦**地面移动无人车**方向，
新增了基于 Gazebo 11 + ROS Noetic 的「无人车激光雷达自主探索建图」模块。

## 本分支新增内容

| 路径 | 说明 |
|------|------|
| `docs/ground/gazebo_tb3_car_explore.md` | 模块文档：原理、运行步骤、实测数据、结论 |
| `docs/img/ground/gazebo_tb3_car_explore/` | 运行截图与分析图表（PNG） |
| `src/gazebo_tb3_car_explore/` | ROS 包源码：launch / 探索控制器 / 数据采集 / RViz 配置 |
| `src/gazebo_tb3_car_explore/data/` | 实测原始数据（CSV / PGM / 模型权重） |

## 选题：无人车

选用 **TurtleBot3 waffle_pi** 差速驱动地面车作为无人车平台，
在 Gazebo 中完成三件事：

1. **物理仿真 + 运动控制**：`/cmd_vel` → 差速驱动 → `/odom`
2. **激光雷达感知 + 控制闭环**：`/scan` → 扇区势场 → `/cmd_vel`
3. **SLAM 建图**：在线占用栅格建图，自主探索覆盖率 2.45%（课程基线 3.3 倍）

## 快速开始

```bash
# 环境：Ubuntu 20.04 + ROS Noetic + Gazebo 11
# 在课程镜像 ros_noetic_humble_gazebov11_linux_win_v1 中已验证

cp -r src/gazebo_tb3_car_explore ~/openhutb_car_ws/src/
cd ~/openhutb_car_ws && catkin_make

source /opt/ros/noetic/setup.bash
source ~/openhutb_car_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# 方式一：launch
roslaunch gazebo_tb3_car_explore explore_map.launch

# 方式二：一键入口（自动启动、统计、保存地图）
python3 $(rospack find gazebo_tb3_car_explore)/main.py
```

## 运行环境

| 项目 | 版本 |
|------|------|
| Ubuntu | 20.04.6 LTS |
| ROS | Noetic (`/opt/ros/noetic`) |
| Gazebo | 11.13.0 |
| Python | 3.8 + numpy 1.24.4 |
| 机器人 | TurtleBot3 waffle_pi |
| 虚拟机 | VMware Workstation 17.6，6 vCPU / 8780 MB |

## 实测结果

| 指标 | 数值 |
|------|------|
| Gazebo 实时因子 | 0.99 ~ 1.00 |
| 渲染帧率 | 52 FPS |
| 前进指令 0.15 m/s → 实测 | 0.1502 m/s |
| 停止后实测速度 | 0.0002 m/s |
| 激光有效回波 | 354 / 360 线 |
| 测距范围 | 0.370 ~ 3.483 m |
| 建图覆盖率 | 2.45%（课程 MLP 基线 0.75%） |

## 文档构建

本仓库沿用上游的 mkdocs 配置：

```bash
pip install -r requirements.txt
mkdocs serve      # 或 Windows 下 serve.bat
```

## 说明

* 本模块不使用 GIF 动图，运行效果均以 PNG 静态截图呈现。
* 所有数据均为在虚拟机内实测采集，保存在 `src/gazebo_tb3_car_explore/data/`。
* 本分支基于 `OpenHUTB/ros2`，遵循其 MIT 许可与文档约定。
