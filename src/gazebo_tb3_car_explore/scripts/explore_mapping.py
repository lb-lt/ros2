#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人车自主探索建图节点.

针对课程 `nn_lidar_avoidance` 的 MLP 在局部极小值（正对障碍时
输出的转向力矩不足以脱离）导致无法持续建图的问题，本节点用经典的
**激光扇区 + 势场逃逸** 策略驱动 TurtleBot3 自主漫游，配合
`lidar_slam.py` 完成大范围建图。

算法（每个控制周期）:
  1. 把 LaserScan 划为 前/左前/右前/左/右 五个扇区，取各扇区最小距离；
  2. 若正前方距离 < 安全阈值 -> 进入"逃逸模式"，原地转向
     距离更大的那一侧，直到正前方距离 >= 逃离阈值；
  3. 否则进入"巡航模式"，以基础线速度前进，并按左右前扇区的
     差值做转向修正（保墙巡游/居中）；
  4. 输出 twist 到 /cmd_vel。

发布:
  /cmd_vel (geometry_msgs/Twist)
订阅:
  /scan    (sensor_msgs/LaserScan)

使用方法::

    rosrun lb2_car_explore explore_mapping.py

参数（rosparam）:
  ~linear_speed   巡航线速度  (默认 0.18 m/s)
  ~turn_speed     原地转向角速度 (默认 0.9 rad/s)
  ~safe_distance  触发逃逸的正前方距离阈值 (默认 0.55 m)
  ~escape_distance 解除逃逸的阈值 (默认 0.85 m)
"""

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Explorer(object):
    """基于激光扇区与势场逃逸的自主探索控制器."""

    def __init__(self):
        self.linear_speed = rospy.get_param('~linear_speed', 0.18)
        self.turn_speed = rospy.get_param('~turn_speed', 0.9)
        self.safe_distance = rospy.get_param('~safe_distance', 0.55)
        self.escape_distance = rospy.get_param('~escape_distance', 0.85)

        self.escaping = False
        self.escape_dir = 1.0  # +1 左转, -1 右转
        self.no_data_stop_logged = False

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)

    NO_RETURN = float('inf')

    def _sector_min(self, msg, a_lo, a_hi):
        """取角度区间 [a_lo, a_hi] 内的最小有效距离.

        关键: 若区间内**没有任何有效回波**, 返回 ``inf`` 而不是一个大数.
        "没有数据" 不等于 "没有障碍" —— 早期的实现把两者混为一谈,
        结果在激光全 inf 时仍然满速前进, 直接把车开出地图.
        """
        best = self.NO_RETURN
        r_max = msg.range_max if msg.range_max > 0 else 12.0
        n = len(msg.ranges)
        for i in range(n):
            a = msg.angle_min + i * msg.angle_increment
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            if a_lo <= a <= a_hi:
                r = msg.ranges[i]
                if math.isinf(r) or math.isnan(r):
                    continue
                if 0.0 < r < best:
                    best = min(r, r_max)
        return best

    def _stop(self):
        self.pub.publish(Twist())

    def on_scan(self, msg):
        front = self._sector_min(msg, -0.35, 0.35)
        front_left = self._sector_min(msg, 0.35, 1.05)
        front_right = self._sector_min(msg, -1.05, -0.35)
        left = self._sector_min(msg, 1.05, 1.75)
        right = self._sector_min(msg, -1.75, -1.05)

        valid = [v for v in (front, front_left, front_right, left, right)
                 if v != self.NO_RETURN]

        twist = Twist()

        # ---- 守卫 1: 整帧几乎无有效回波 -> 认为传感器异常/已出界, 停车 ----
        if not valid:
            if not self.no_data_stop_logged:
                rospy.logwarn('no valid scans in any sector -> hold stop')
                self.no_data_stop_logged = True
            self._stop()
            return
        self.no_data_stop_logged = False

        # 用有限扇区补全缺失值, 避免 inf 参与比较时被当成"空旷"
        fallback = max(valid)
        front = fallback if front == self.NO_RETURN else front
        front_left = fallback if front_left == self.NO_RETURN else front_left
        front_right = fallback if front_right == self.NO_RETURN else front_right
        left = fallback if left == self.NO_RETURN else left
        right = fallback if right == self.NO_RETURN else right

        # ---- 守卫 2: 逃逸模式 ----
        if self.escaping:
            if front >= self.escape_distance:
                self.escaping = False
            else:
                twist.angular.z = self.escape_dir * self.turn_speed
                self.pub.publish(twist)
                return

        # ---- 守卫 3: 正前方过近 -> 进入逃逸 ----
        if front < self.safe_distance:
            self.escaping = True
            left_room = min(front_left, left)
            right_room = min(front_right, right)
            self.escape_dir = 1.0 if left_room >= right_room else -1.0
            twist.angular.z = self.escape_dir * self.turn_speed
            self.pub.publish(twist)
            return

        # ---- 巡航: 按左右前扇区差值居中, 距离越近越减速 ----
        scale = max(0.0, min(1.0, (front - self.safe_distance) / 0.8))
        twist.linear.x = self.linear_speed * scale
        diff = front_left - front_right
        twist.angular.z = max(-0.6, min(0.6, 0.9 * diff))
        self.pub.publish(twist)

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node('explore_mapping', anonymous=False)
    explorer = Explorer()
    rospy.loginfo('explore_mapping 启动: linear=%.2f turn=%.2f safe=%.2f escape=%.2f',
                  explorer.linear_speed, explorer.turn_speed,
                  explorer.safe_distance, explorer.escape_distance)
    try:
        explorer.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
