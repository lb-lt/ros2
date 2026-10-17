#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采集一次无人车运行数据，保存为 CSV.

记录内容:
    /scan  -> 最小/平均测距
    /cmd_vel -> 线速度、角速度
    /odom  -> 位姿与真实速度

用法::

    rosrun gazebo_tb3_car_explore record_run.py --duration 30 --out data/run.csv
"""
from __future__ import print_function

import argparse
import csv
import math
import os

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Recorder(object):

    def __init__(self):
        self.scan = None
        self.cmd = None
        self.odom = None
        rospy.Subscriber('/scan', LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self._on_cmd, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self._on_odom, queue_size=10)

    def _on_scan(self, msg):
        vals = [r for r in msg.ranges
                if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        if vals:
            self.scan = (min(vals), sum(vals) / len(vals))
        else:
            self.scan = (float('nan'), float('nan'))

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        t = msg.twist.twist
        self.odom = (p.x, p.y, t.linear.x, t.angular.z)

    def row(self, idx):
        s = self.scan or (float('nan'),) * 2
        c = self.cmd or (float('nan'),) * 2
        o = self.odom or (float('nan'),) * 4
        return [idx, s[0], s[1], c[0], c[1], o[0], o[1], o[2], o[3]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--rate', type=float, default=0.3, help='采样间隔秒')
    ap.add_argument('--out', default='run.csv')
    args = ap.parse_args()

    rospy.init_node('record_run', anonymous=True)
    rec = Recorder()

    out = args.out
    d = os.path.dirname(out)
    if d and not os.path.isdir(d):
        os.makedirs(d)

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['index', 'min_range', 'mean_range',
                    'cmd_linear_x', 'cmd_angular_z',
                    'odom_x', 'odom_y', 'odom_vx', 'odom_wz'])
        n = 0
        t_end = rospy.get_time() + args.duration
        while not rospy.is_shutdown() and rospy.get_time() < t_end:
            w.writerow(rec.row(n))
            n += 1
            rospy.sleep(args.rate)
    print('已保存 %d 行 -> %s' % (n, out))


if __name__ == '__main__':
    main()
