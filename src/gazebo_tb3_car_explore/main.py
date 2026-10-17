#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo 无人车激光雷达自主探索建图 —— 一键入口.

用法::

    python3 main.py              # 启动仿真 + 建图 + 统计
    python3 main.py --no-launch  # 只做统计（仿真已在运行）

流程:
    1. 检查 ROS 环境与关键包；
    2. 启动 explore_map.launch（子进程）；
    3. 等待 /map 话题出现并积累一段时间；
    4. 统计占用栅格覆盖率，保存地图与轨迹；
    5. 打印结果摘要，等待 Ctrl-C 退出。
"""
from __future__ import print_function

import argparse
import os
import subprocess
import sys
import time

PKG = 'gazebo_tb3_car_explore'
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, 'data')


def check_env():
    """检查 ROS Noetic 与依赖包是否就绪."""
    print('== 1. 检查运行环境 ==')
    if not os.environ.get('ROS_DISTRO'):
        # 未 source，尝试自动 source
        setup = '/opt/ros/noetic/setup.bash'
        if os.path.exists(setup):
            print('  ROS_DISTRO 未设置，请先执行: source %s' % setup)
        print('  [警告] 未检测到 ROS 环境变量 ROS_DISTRO')
        return False

    print('  ROS_DISTRO = %s' % os.environ['ROS_DISTRO'])
    ok = True
    for pkg in ('gazebo_ros', 'turtlebot3_gazebo', 'turtlebot3_description'):
        r = subprocess.Popen(
            ['rospack', 'find', pkg],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, _ = r.communicate()
        if r.returncode == 0:
            print('  [OK]   %-24s %s' % (pkg, out.decode().strip()))
        else:
            print('  [缺失] %-24s' % pkg)
            ok = False
    return ok


def launch_sim():
    """启动仿真（后台子进程）."""
    print('\n== 2. 启动 Gazebo 仿真与自主探索 ==')
    env = dict(os.environ)
    env.setdefault('TURTLEBOT3_MODEL', 'waffle_pi')
    proc = subprocess.Popen(
        ['roslaunch', PKG, 'explore_map.launch'],
        env=env)
    print('  roslaunch PID = %d' % proc.pid)
    return proc


def wait_for_map(timeout=120):
    """等待 /map 话题出现."""
    import rospy
    print('\n== 3. 等待 /map 话题 ==')
    t0 = time.time()
    while time.time() - t0 < timeout:
        topics = rospy.get_published_topics()
        if any(t[0] == '/map' for t in topics):
            print('  /map 已出现（耗时 %.1f s）' % (time.time() - t0))
            return True
        time.sleep(2)
    print('  [超时] 未等到 /map')
    return False


def stats_and_save(wait_s=60):
    """统计覆盖率并保存地图."""
    import numpy as np
    import rospy
    from nav_msgs.msg import OccupancyGrid

    print('\n== 4. 统计建图覆盖率（积累 %d s）==' % wait_s)
    rospy.sleep(wait_s)

    grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=30)
    data = np.asarray(grid.data, dtype=np.int16)
    info = grid.info
    free = int((data == 0).sum())
    occupied = int((data == 100).sum())
    unknown = int((data == -1).sum())
    total = data.size
    ratio = (free + occupied) / float(total) if total else 0.0

    print('  地图尺寸 %d x %d，分辨率 %.3f m/格' % (info.width, info.height, info.resolution))
    print('  未知 %d / 空闲 %d / 占据 %d，共 %d' % (unknown, free, occupied, total))
    print('  探索覆盖率 = %.4f (%.2f%%)' % (ratio, ratio * 100))

    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)
    pgm = os.path.join(OUT_DIR, 'map_run.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (info.width, info.height))
        f.write(bytes((0 if v > 50 else (255 if v < 0 else 205)) for v in data))
    print('  地图已保存: %s' % pgm)
    return ratio


def main():
    ap = argparse.ArgumentParser(description='Gazebo 无人车自主探索建图入口')
    ap.add_argument('--no-launch', action='store_true', help='不启动仿真，仅统计')
    ap.add_argument('--duration', type=int, default=120,
                    help='统计前积累时长(秒)；默认 120s，与文档实测覆盖率对应')
    args = ap.parse_args()

    if not args.no_launch:
        if not check_env():
            print('\n环境检查未通过，仍尝试继续...')
        proc = launch_sim()
    else:
        proc = None
        print('== 跳过启动，直接统计 ==')

    import rospy
    rospy.init_node('gazebo_tb3_car_explore_main', anonymous=True)
    try:
        if wait_for_map():
            stats_and_save(args.duration)
    except rospy.ROSInterruptException:
        pass
    finally:
        if proc is not None:
            print('\n关闭仿真进程...')
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except Exception:
                proc.kill()
    print('\n完成。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
