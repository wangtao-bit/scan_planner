#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace scan_planner
{
// 实机位姿坐标轴可视化：里程计回调(100Hz)只缓存位姿，定时器按 publish_rate 降频发布 Marker
class RobotPoseVisualizer : public rclcpp::Node
{
public:
  RobotPoseVisualizer() : Node("robot_pose_visualizer")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const double publish_rate = std::max(1.0, declare_parameter<double>("publish_rate", 10.0));
    const double axis_length = declare_parameter<double>("axis_length", 0.3);
    const double axis_width = declare_parameter<double>("axis_width", 0.02);

    // 坐标轴几何与颜色固定不变，只构造一次，定时器里仅更新 pose 和 stamp
    marker_.ns = "robot_axes";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = axis_width;
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
    marker_.points = {point(0, 0, 0), point(axis_length, 0, 0),
                      point(0, 0, 0), point(0, axis_length, 0),
                      point(0, 0, 0), point(0, 0, axis_length)};
    marker_.colors = {color(1, 0, 0), color(1, 0, 0),
                      color(0, 1, 0), color(0, 1, 0),
                      color(0, 0, 1), color(0, 0, 1)};

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot_axes", 1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&RobotPoseVisualizer::odomCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                               std::bind(&RobotPoseVisualizer::publishMarker, this));
    RCLCPP_INFO(get_logger(), "Robot pose visualizer publishing at %.1f Hz", publish_rate);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    // 单线程执行器下回调与定时器不会并发，无需加锁
    marker_.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    marker_.pose = msg->pose.pose;
    has_pose_ = true;
  }

  void publishMarker()
  {
    if (!has_pose_ || marker_pub_->get_subscription_count() == 0) return;
    marker_.header.stamp = now();
    marker_pub_->publish(marker_);
  }

  std::string frame_id_;
  bool has_pose_{false};
  visualization_msgs::msg::Marker marker_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
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
