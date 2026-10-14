#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace scan_planner
{
// 实机位姿坐标轴可视化：里程计回调(100Hz)只缓存位姿，定时器按 publish_rate 降频发布 Marker 和 TF
// RViz 里 TF 显示自带 Marker Scale 滑条，可实时调节坐标轴大小
class RobotPoseVisualizer : public rclcpp::Node
{
public:
  RobotPoseVisualizer() : Node("robot_pose_visualizer")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    tf_child_frame_ = declare_parameter<std::string>("tf_child_frame", "robot_base");
    const double publish_rate = std::max(1.0, declare_parameter<double>("publish_rate", 10.0));
    const double axis_length = declare_parameter<double>("axis_length", 0.3);
    const double axis_width = declare_parameter<double>("axis_width", 0.02);

    // 坐标轴几何与颜色固定不变，只构造一次，定时器里仅更新 pose 和 stamp
    marker_.header.frame_id = frame_id_;  // 固定用规划坐标系(world)，不继承 /lio_odom 的 map，避免 RViz 无 TF 丢弃
    marker_.ns = "robot_axes";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = axis_width;
    marker_.pose.orientation.w = 1.0;  // 初始化为单位四元数
    
    const auto point = [](double x, double y, double z) {
      geometry_msgs::msg::Point p;
      p.x = x; p.y = y; p.z = z;
      return p;
    };
    const auto color = [](float r, float g, float b) {
      std_msgs::msg::ColorRGBA c;
      c.r = r; c.g = g; c.b = b; c.a = 1.0f;
      return c;
    };
    // RGB 轴：原点到各轴端点
    marker_.points = {point(0, 0, 0), point(axis_length, 0, 0),
                      point(0, 0, 0), point(0, axis_length, 0),
                      point(0, 0, 0), point(0, 0, axis_length)};
    marker_.colors = {color(1, 0, 0), color(1, 0, 0),
                      color(0, 1, 0), color(0, 1, 0),
                      color(0, 0, 1), color(0, 0, 1)};

    // Marker publisher 使用 TransientLocal 持久化，队列深度 1
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "robot_axes", rclcpp::QoS(1).transient_local());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&RobotPoseVisualizer::odomCallback, this, std::placeholders::_1));
    
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                               std::bind(&RobotPoseVisualizer::publishMarker, this));
    
    RCLCPP_INFO(get_logger(), "Robot pose visualizer: frame=%s, rate=%.1fHz", 
                frame_id_.c_str(), publish_rate);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    // 只缓存位姿，frame_id 始终使用参数配置的固定值
    marker_.pose = msg->pose.pose;
    has_pose_ = true;
  }

  void publishMarker()
  {
    if (!has_pose_) return;
    // 即使没有订阅者也发布（TransientLocal 会保留最后一条消息）
    marker_.header.stamp = now();
    marker_pub_->publish(marker_);

    // 广播 world -> robot_base TF，配合 RViz TF 显示的 Marker Scale 滑条调大小
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = marker_.header.stamp;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = tf_child_frame_;
    tf_msg.transform.translation.x = marker_.pose.position.x;
    tf_msg.transform.translation.y = marker_.pose.position.y;
    tf_msg.transform.translation.z = marker_.pose.position.z;
    tf_msg.transform.rotation = marker_.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string frame_id_;
  std::string tf_child_frame_;
  bool has_pose_{false};
  visualization_msgs::msg::Marker marker_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::RobotPoseVisualizer>());
  rclcpp::shutdown();
  return 0;
}
