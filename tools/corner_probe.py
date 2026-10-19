#!/usr/bin/env python3
"""Record a Gazebo run and measure how close the Go2 body gets to walls.

Usage (sim already running):
  python3 corner_probe.py --goal 1.0 1.5 --duration 90 --out /tmp/corner_run1

Logs per body_pose sample:
  pose, yaw, p3d twist, finite-difference velocity (to check twist frame),
  last cmd_vel (requested vs what CHAMP will actually allow),
  lateral deviation from the latest planned B-spline,
  min distance from the Go2 footprint rectangle to every wall / static obstacle
  parsed from the .world file (negative => penetrating geometry).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from scipy.interpolate import BSpline

DEFAULT_WORLD = os.path.expanduser(
    "~/go2_sim_ws/install/go2_sim_bringup/share/go2_sim_bringup/worlds/indoor_complex.world")

# Go2 outer footprint in base_link (trunk + hips + feet incl. stride), metres.
FOOT_HALF_L = 0.30
FOOT_HALF_W = 0.17
# CHAMP gait.yaml caps (per axis, applied independently).
CHAMP_VX, CHAMP_VY, CHAMP_WZ = 0.30, 0.25, 0.50


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rect(cx, cy, hl, hw, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw)]
    return np.array([[cx + c * x - s * y, cy + s * x + c * y] for x, y in pts])


def circle_poly(cx, cy, r, n=16):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def _seg_pt(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t * ab))


def _overlap(P, Q):
    for poly in (P, Q):
        n = len(poly)
        for i in range(n):
            e = poly[(i + 1) % n] - poly[i]
            ax = np.array([-e[1], e[0]])
            pp, qq = P @ ax, Q @ ax
            if pp.max() < qq.min() or qq.max() < pp.min():
                return False
    return True


def poly_dist(P, Q):
    """Signed-ish distance: >0 gap, <=0 overlap (returns -penetration estimate)."""
    if _overlap(P, Q):
        # crude penetration depth: min over SAT axes of overlap length
        best = 1e9
        for poly in (P, Q):
            n = len(poly)
            for i in range(n):
                e = poly[(i + 1) % n] - poly[i]
                ax = np.array([-e[1], e[0]])
                ax /= max(np.linalg.norm(ax), 1e-12)
                pp, qq = P @ ax, Q @ ax
                best = min(best, min(pp.max() - qq.min(), qq.max() - pp.min()))
        return -best
    d = 1e9
    for A, B in ((P, Q), (Q, P)):
        n = len(B)
        for p in A:
            for i in range(n):
                d = min(d, _seg_pt(p, B[i], B[(i + 1) % n]))
    return d


def parse_pose(text):
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    v += [0.0] * (6 - len(v))
    return v


def load_obstacles(world_path):
    """Return list of (name, polygon) for every box/cylinder collision above the floor."""
    root = ET.parse(world_path).getroot()
    obs = []
    for model in root.iter("model"):
        mname = model.get("name")
        if mname == "ground_plane":
            continue
        mp = parse_pose(model.findtext("pose"))
        for col in model.iter("collision"):
            cp = parse_pose(col.findtext("pose"))
            cx = mp[0] + math.cos(mp[5]) * cp[0] - math.sin(mp[5]) * cp[1]
            cy = mp[1] + math.sin(mp[5]) * cp[0] + math.cos(mp[5]) * cp[1]
            cz = mp[2] + cp[2]
            yaw = mp[5] + cp[5]
            box = col.find("geometry/box/size")
            cyl = col.find("geometry/cylinder")
            if box is not None:
                sx, sy, sz = [float(x) for x in box.text.split()]
                if cz + sz / 2 < 0.05:  # floor slab
                    continue
                obs.append((f"{mname}/{col.get('name')}", rect(cx, cy, sx / 2, sy / 2, yaw)))
            elif cyl is not None:
                r = float(cyl.findtext("radius"))
                obs.append((f"{mname}/{col.get('name')}", circle_poly(cx, cy, r)))
    return obs


class Probe(Node):
    def __init__(self, args):
        super().__init__("corner_probe")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.obs = load_obstacles(args.world)
        self.get_logger().info(f"loaded {len(self.obs)} obstacle polygons from {args.world}")
        self.cmd = Twist()
        self.spline = None
        self.spline_pts = None
        self.prev = None
        self.rows = []
        self.out = args.out
        self.create_subscription(Odometry, "/quad_0/body_pose", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/quad_0/cmd_vel", self.on_cmd, 20)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 1)
        self.goal = args.goal
        self.goal_sent = False
        self.create_timer(1.0, self.send_goal_once)

    def send_goal_once(self):
        if self.goal_sent or self.goal is None or self.prev is None:
            return
        if self.goal_pub.get_subscription_count() == 0:
            return
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.goal
        msg.pose.position.z = 0.3
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.goal_sent = True
        self.get_logger().info(f"goal sent: {self.goal}")

    def on_cmd(self, msg):
        self.cmd = msg

    def on_bspline(self, msg):
        k = msg.order
        ctrl = np.array([[p.x, p.y] for p in msg.pos_pts])
        knots = np.array(msg.knots)
        try:
            spl = BSpline(knots, ctrl, k)
            t = np.linspace(knots[k], knots[-k - 1], 300)
            self.spline_pts = spl(t)
            self.spline_ctrl = ctrl
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"bspline eval failed: {exc}")

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        tw = msg.twist.twist.linear
        fd_vx = fd_vy = float("nan")
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            fd_vx = (p.x - self.prev[1]) / dt
            fd_vy = (p.y - self.prev[2]) / dt
        self.prev = (t, p.x, p.y)

        foot = rect(p.x, p.y, FOOT_HALF_L, FOOT_HALF_W, yaw)
        dmin, who = 1e9, ""
        for name, poly in self.obs:
            d = poly_dist(foot, poly)
            if d < dmin:
                dmin, who = d, name
        dev = float("nan")
        if self.spline_pts is not None:
            dev = float(np.min(np.linalg.norm(self.spline_pts - np.array([p.x, p.y]), axis=1)))
        c = self.cmd
        self.rows.append(dict(
            t=t, x=p.x, y=p.y, yaw=yaw,
            tw_x=tw.x, tw_y=tw.y, fd_vx=fd_vx, fd_vy=fd_vy,
            cmd_vx=c.linear.x, cmd_vy=c.linear.y, cmd_wz=c.angular.z,
            champ_vx=max(-CHAMP_VX, min(CHAMP_VX, c.linear.x)),
            champ_vy=max(-CHAMP_VY, min(CHAMP_VY, c.linear.y)),
            champ_wz=max(-CHAMP_WZ, min(CHAMP_WZ, c.angular.z)),
            path_dev=dev, wall_clear=dmin, nearest=who,
        ))

    def dump(self):
        os.makedirs(self.out, exist_ok=True)
        path = os.path.join(self.out, "samples.csv")
        if not self.rows:
            print("no samples")
            return
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        if self.spline_pts is not None:
            np.savetxt(os.path.join(self.out, "last_spline.csv"), self.spline_pts, delimiter=",")
        print(f"wrote {len(self.rows)} samples → {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", type=float, nargs=2, default=None)
    ap.add_argument("--duration", type=float, default=60.0, help="wall seconds")
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--out", default="/tmp/corner_probe")
    args = ap.parse_args()
    rclpy.init()
    node = Probe(args)
    import time
    t_end = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.dump()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
