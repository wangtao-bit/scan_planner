#!/usr/bin/env python3
"""Bridge Gazebo Go2 (CHAMP / go2_sim_ws) topics onto SCAN-Planner interfaces.

Gazebo side (defaults match go2_sim_bringup + velodyne):
  /odom/ground_truth  nav_msgs/Odometry
  /velodyne_points    sensor_msgs/PointCloud2
  /cmd_vel            geometry_msgs/Twist  (consumed by CHAMP)

Planner side (same names as the old kinematic simulator):
  /quad_0/body_pose   nav_msgs/Odometry
  /quad_0/lidar_pose  nav_msgs/Odometry   (sensor_pose for grid_map)
  /quad_0/cloud       sensor_msgs/PointCloud2
  /quad_0/cmd_vel     geometry_msgs/Twist  (from closed_loop_controller)

Point clouds stay in the sensor frame; lidar_pose is filled from TF
(cloud.frame_id -> odom frame) so grid_map can run with
cloud_is_world:=false and need_extrinsic:=false.

IMPORTANT: lidar_pose must NOT be filled with body odom. base_link→velodyne
is ~0.2 m in x; publishing body as sensor_pose makes grid_map alternate
ray origins and the inflate map jitters.

ready_topic (std_msgs/Bool, latched): when set, nothing is forwarded to the
planner until it reports true. The rl_sar backend drops the dog in limp and
stands it up first; scan_replan_fsm latches the RViz goal height from the
first body_pose, so it must not see the robot mid-fall.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class GazeboSimBridge(Node):
    def __init__(self):
        super().__init__("gazebo_sim_bridge")

        self.declare_parameter("odom_topic", "/odom/ground_truth")
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("gazebo_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("body_pose_topic", "/quad_0/body_pose")
        self.declare_parameter("sensor_pose_topic", "/quad_0/lidar_pose")
        self.declare_parameter("planner_cloud_topic", "/quad_0/cloud")
        self.declare_parameter("planner_cmd_vel_topic", "/quad_0/cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("force_odom_frame_id", True)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("ready_topic", "")

        odom_topic = self.get_parameter("odom_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value
        gazebo_cmd = self.get_parameter("gazebo_cmd_vel_topic").value
        body_topic = self.get_parameter("body_pose_topic").value
        sensor_topic = self.get_parameter("sensor_pose_topic").value
        planner_cloud = self.get_parameter("planner_cloud_topic").value
        planner_cmd = self.get_parameter("planner_cmd_vel_topic").value

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._force_frame = bool(self.get_parameter("force_odom_frame_id").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_odom: Odometry | None = None
        self._tf_stamp_ok = 0
        self._tf_latest_ok = 0
        self._tf_fail = 0

        # world->odom is published ONLY by run.launch's static_transform_publisher.
        # Do not also publish it here: same edge on /tf + /tf_static with different
        # stamps causes RViz RobotModel/TF to flicker.

        self._body_pub = self.create_publisher(Odometry, body_topic, 50)
        self._sensor_pub = self.create_publisher(Odometry, sensor_topic, 50)
        self._cloud_pub = self.create_publisher(PointCloud2, planner_cloud, qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Twist, gazebo_cmd, 20)

        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, planner_cmd, self._on_cmd, 20)
        self.create_timer(5.0, self._log_tf_stats)

        ready_topic = str(self.get_parameter("ready_topic").value)
        self._ready = not ready_topic
        if ready_topic:
            self.create_subscription(
                Bool, ready_topic, self._on_ready,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.get_logger().info(f"holding planner topics until {ready_topic} is true")

        self.get_logger().info(
            f"Gazebo↔planner bridge ready: odom={odom_topic} cloud={cloud_topic} "
            f"→ {body_topic}/{sensor_topic}/{planner_cloud}; "
            f"{planner_cmd} → {gazebo_cmd}"
        )

    def _on_ready(self, msg: Bool):
        if msg.data and not self._ready:
            self.get_logger().info("robot ready, forwarding odom/cloud to the planner")
        self._ready = bool(msg.data)

    def _on_odom(self, msg: Odometry):
        if not self._ready:
            return
        out = Odometry()
        out.header = msg.header
        if self._force_frame:
            out.header.frame_id = self._odom_frame
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self._last_odom = out
        self._body_pub.publish(out)
        # Do NOT publish body as lidar_pose. Velodyne is ~0.2 m ahead of
        # base_link; grid_map would alternate ray origins and inflate jitter.

    def _on_cloud(self, msg: PointCloud2):
        if not self._ready:
            return
        sensor_odom = self._sensor_odom_from_tf(msg)
        if sensor_odom is None:
            # Drop this cloud rather than projecting with a wrong body pose.
            return

        # Publish pose before cloud so grid_map's latest-pose path is more likely
        # to see the matching lidar frame (lidar path has no ApproximateTime sync).
        self._sensor_pub.publish(sensor_odom)

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = msg.height
        cloud.width = msg.width
        cloud.fields = msg.fields
        cloud.is_bigendian = msg.is_bigendian
        cloud.point_step = msg.point_step
        cloud.row_step = msg.row_step
        cloud.data = msg.data
        cloud.is_dense = msg.is_dense
        self._cloud_pub.publish(cloud)

    def _sensor_odom_from_tf(self, cloud: PointCloud2) -> Odometry | None:
        if not cloud.header.frame_id:
            return None

        source = "stamp"
        try:
            tf = self._tf_buffer.lookup_transform(
                self._odom_frame,
                cloud.header.frame_id,
                cloud.header.stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
            self._tf_stamp_ok += 1
        except Exception:
            try:
                # Prefer latest TF over body odom: keeps base→velodyne lever arm.
                tf = self._tf_buffer.lookup_transform(
                    self._odom_frame,
                    cloud.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self._tf_timeout),
                )
                self._tf_latest_ok += 1
                source = "latest"
            except Exception as exc:
                self._tf_fail += 1
                self.get_logger().warn(
                    f"TF {self._odom_frame} <- {cloud.header.frame_id} unavailable, "
                    f"dropping cloud (not falling back to body): {exc}",
                    throttle_duration_sec=2.0,
                )
                return None

        if source == "latest":
            self.get_logger().warn(
                f"TF stamp miss for cloud@{cloud.header.stamp.sec}."
                f"{cloud.header.stamp.nanosec:09d}; using latest "
                f"{self._odom_frame}<-{cloud.header.frame_id}",
                throttle_duration_sec=5.0,
            )

        out = Odometry()
        out.header.stamp = cloud.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = cloud.header.frame_id
        out.pose.pose.position.x = tf.transform.translation.x
        out.pose.pose.position.y = tf.transform.translation.y
        out.pose.pose.position.z = tf.transform.translation.z
        out.pose.pose.orientation = tf.transform.rotation
        if self._last_odom is not None:
            out.twist = self._last_odom.twist
        return out

    def _log_tf_stats(self):
        total = self._tf_stamp_ok + self._tf_latest_ok + self._tf_fail
        if total == 0:
            return
        self.get_logger().info(
            f"lidar_pose TF stats (5s): "
            f"stamp_ok={self._tf_stamp_ok} latest_fallback={self._tf_latest_ok} "
            f"fail_drop={self._tf_fail}"
        )
        self._tf_stamp_ok = 0
        self._tf_latest_ok = 0
        self._tf_fail = 0

    def _on_cmd(self, msg: Twist):
        self._cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = GazeboSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
