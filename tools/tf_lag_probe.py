#!/usr/bin/env python3
"""Measure how stale the odom->base_link TF is when each Velodyne cloud arrives.

For every /velodyne_points message, report (latest TF stamp - cloud stamp)
and the yaw change the robot made in that interval (→ wall smear in the map).
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class P(Node):
    def __init__(self):
        super().__init__("tf_lag_probe")
        self.tf_hist = []  # (stamp, yaw)
        self.rows = []
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(PointCloud2, "/velodyne_points", self.on_cloud, qos_profile_sensor_data)

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                self.tf_hist.append((stamp(t.header.stamp), yaw_of(t.transform.rotation)))
                self.tf_hist = self.tf_hist[-200:]

    def on_cloud(self, msg):
        if len(self.tf_hist) < 2:
            return
        tc = stamp(msg.header.stamp)
        tl, yl = self.tf_hist[-1]
        ts = np.array([h[0] for h in self.tf_hist])
        ys = np.unwrap(np.array([h[1] for h in self.tf_hist]))
        y_at_cloud = np.interp(tc, ts, ys) if ts[0] <= tc <= ts[-1] else float("nan")
        self.rows.append((tl - tc, abs(ys[-1] - y_at_cloud), tc <= ts[-1]))


def main():
    rclpy.init()
    n = P()
    t_end = time.time() + 20
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    r = np.array([(a, b, c) for a, b, c in n.rows], dtype=float)
    if len(r) == 0:
        print("no data")
        return
    lag = r[:, 0]
    print(f"clouds={len(r)}  latestTF - cloudStamp: median={np.median(lag)*1000:.0f} ms  "
          f"min={lag.min()*1000:.0f}  max={lag.max()*1000:.0f} ms")
    print(f"cloud stamp already covered by TF buffer at arrival: {r[:,2].mean()*100:.0f}%")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
