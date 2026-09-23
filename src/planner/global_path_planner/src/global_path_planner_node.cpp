#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "global_path_planner/support_astar.hpp"
#include "global_path_planner/support_map.hpp"

namespace global_path_planner
{

class GlobalPathPlannerNode : public rclcpp::Node
{
public:
  GlobalPathPlannerNode()
  : Node("global_path_planner_node")
  {
    declare_parameter<std::string>("pcd_file", "");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<double>("min_clearance", 1.0);
    declare_parameter<double>("robot_radius", 0.25);
    declare_parameter<double>("robot_height", 0.4);
    declare_parameter<double>("max_step_height", 0.40);
    declare_parameter<double>("xy_snap_radius", 1.5);
    declare_parameter<double>("goal_z_tolerance", 0.6);
    declare_parameter<double>("edge_cost_weight", 0.4);
    declare_parameter<double>("path_min_distance", 0.5);
    declare_parameter<bool>("use_odom_as_start", true);
    declare_parameter<bool>("plan_on_startup", false);
    declare_parameter<bool>("publish_support_cloud", true);
    declare_parameter<std::vector<double>>("startup_start", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("startup_goal", {1.0, 0.0, 0.0});

    frame_id_ = get_parameter("frame_id").as_string();
    use_odom_as_start_ = get_parameter("use_odom_as_start").as_bool();
    path_min_distance_ = get_parameter("path_min_distance").as_double();
    publish_support_cloud_ = get_parameter("publish_support_cloud").as_bool();

    SupportMapParams mp;
    mp.resolution = get_parameter("resolution").as_double();
    mp.min_clearance = get_parameter("min_clearance").as_double();
    mp.robot_radius = get_parameter("robot_radius").as_double();
    mp.robot_height = get_parameter("robot_height").as_double();
    mp.max_step_height = get_parameter("max_step_height").as_double();
    mp.xy_snap_radius = get_parameter("xy_snap_radius").as_double();
    mp.goal_z_tolerance = get_parameter("goal_z_tolerance").as_double();
    mp.edge_cost_weight = get_parameter("edge_cost_weight").as_double();

    const std::string pcd_file = get_parameter("pcd_file").as_string();
    std::string err;
    if (pcd_file.empty() || !map_.buildFromPcd(pcd_file, mp, &err)) {
      RCLCPP_FATAL(get_logger(), "Failed to build support map: %s", err.c_str());
      throw std::runtime_error(err.empty() ? "pcd_file empty or build failed" : err);
    }
    planner_.setMap(&map_);
    RCLCPP_INFO(
      get_logger(), "Support map ready: %dx%dx%d @ %.2fm from %s", map_.size().x(),
      map_.size().y(), map_.size().z(), map_.resolution(), pcd_file.c_str());

    auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("initial_path", path_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("global_path_marker", 10);
    support_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "support_map", rclcpp::QoS(1).reliable().transient_local());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 10, std::bind(&GlobalPathPlannerNode::onGoal, this, std::placeholders::_1));
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 10,
      std::bind(&GlobalPathPlannerNode::onClickedPoint, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GlobalPathPlannerNode::onOdom, this, std::placeholders::_1));

    if (publish_support_cloud_) {
      publishSupportCloud();
    }

    if (get_parameter("plan_on_startup").as_bool()) {
      const auto start = get_parameter("startup_start").as_double_array();
      const auto goal = get_parameter("startup_goal").as_double_array();
      if (start.size() >= 3 && goal.size() >= 3) {
        runPlan(
          Eigen::Vector3d(start[0], start[1], start[2]),
          Eigen::Vector3d(goal[0], goal[1], goal[2]));
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Waiting for goals on 'goal' (/move_base_simple/goal) or 'clicked_point' "
      "(/clicked_point); publishing 'initial_path'");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  bool resolveStart(Eigen::Vector3d *start)
  {
    if (!start) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (use_odom_as_start_ && have_odom_) {
      *start = Eigen::Vector3d(
        latest_odom_.pose.pose.position.x, latest_odom_.pose.pose.position.y,
        latest_odom_.pose.pose.position.z);
      // Mode 3 path z is ground height; odom is typically body height.
      start->z() -= map_.params().robot_height;
      return true;
    }
    const auto s = get_parameter("startup_start").as_double_array();
    if (s.size() < 3) {
      RCLCPP_WARN(get_logger(), "No odom and no startup_start; ignore goal");
      return false;
    }
    *start = Eigen::Vector3d(s[0], s[1], s[2]);
    return true;
  }

  void planToGoal(const Eigen::Vector3d &goal)
  {
    Eigen::Vector3d start;
    if (!resolveStart(&start)) return;
    runPlan(start, goal);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // RViz 2D Goal Pose usually sets z=0; snap will pick nearby support.
    planToGoal(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // RViz Publish Point carries 3D coords (e.g. click on /support_map).
    RCLCPP_INFO(
      get_logger(), "Clicked point goal (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y,
      msg->point.z);
    planToGoal(Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z));
  }

  void runPlan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
  {
    const PlanResult result = planner_.plan(start, goal);
    RCLCPP_INFO(
      get_logger(), "Plan %s: %s", planStatusToString(result.status), result.message.c_str());
    if (result.status != PlanStatus::Success) return;

    auto path_pts = SupportAstar::downsample(result.path, path_min_distance_);
    if (path_pts.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Path too short after downsample");
      return;
    }
    publishPath(path_pts);
  }

  void publishPath(const std::vector<Eigen::Vector3d> &pts)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = now();
    path.poses.reserve(pts.size());
    for (const auto &p : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);

    visualization_msgs::msg::Marker mk;
    mk.header = path.header;
    mk.ns = "global_path";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = 0.08;
    mk.color.r = 0.1f;
    mk.color.g = 0.9f;
    mk.color.b = 0.2f;
    mk.color.a = 1.0f;
    mk.pose.orientation.w = 1.0;
    for (const auto &p : pts) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      mk.points.push_back(gp);
    }
    marker_pub_->publish(mk);
    RCLCPP_INFO(get_logger(), "Published %zu waypoints on initial_path", pts.size());
  }

  void publishSupportCloud()
  {
    const auto pts = map_.supportCloud();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(pts.size());
    for (const auto &p : pts) cloud.emplace_back(p.x(), p.y(), p.z());
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    support_pub_->publish(msg);
  }

  SupportMap map_;
  SupportAstar planner_;
  std::string frame_id_;
  bool use_odom_as_start_{true};
  bool have_odom_{false};
  bool publish_support_cloud_{true};
  double path_min_distance_{0.5};
  nav_msgs::msg::Odometry latest_odom_;
  std::mutex mutex_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace global_path_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<global_path_planner::GlobalPathPlannerNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("global_path_planner_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
