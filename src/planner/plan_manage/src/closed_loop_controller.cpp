#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    ki_pos_ = declare_parameter<double>("ki_pos", 0.15);
    kd_pos_ = declare_parameter<double>("kd_pos", 0.3);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);

    // 新增参数
    integral_limit_ = declare_parameter<double>("integral_limit", 0.5);
    adaptive_time_gain_ = declare_parameter<double>("adaptive_time_gain", 2.0);
    heading_speed_decay_ = declare_parameter<double>("heading_speed_decay", 0.3);
    use_acceleration_feedforward_ = declare_parameter<bool>("use_acceleration_feedforward", true);
    feedforward_priority_ = declare_parameter<double>("feedforward_priority", 0.7);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready with PID+adaptive control");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    // 基于速度的自适应前瞻距离
    const double current_speed = std::max(0.1, odom_vel_.head<2>().norm());
    const double adaptive_forward = time_forward_ * current_speed / max_vx_;
    const double t_look = std::min(traj_duration_, t_cur + adaptive_forward);

    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;

    // 重置积分项
    pos_error_integral_ = Eigen::Vector2d::Zero();
    last_pos_error_ = Eigen::Vector2d::Zero();

    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;

    // 转换到世界坐标系
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    odom_vel_world_.x() = c * odom_vel_.x() - s * odom_vel_.y();
    odom_vel_world_.y() = s * odom_vel_.x() + c * odom_vel_.y();

    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;

    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);

    // 柔性航向约束：用速度衰减代替硬停止
    double speed_scale = 1.0;
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      // 线性衰减到 heading_speed_decay_
      const double excess = std::abs(yaw_error) - heading_error_threshold_;
      speed_scale = std::max(heading_speed_decay_,
                            1.0 - excess / (M_PI - heading_error_threshold_));
      publishExecutionFrozen(true);
    }
    else
    {
      publishExecutionFrozen(false);
    }

    // 计算位置误差
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());

    // 自适应时间推进：误差大时减速
    const double error_norm = pos_error.norm();
    const double time_scale = 1.0 / (1.0 + adaptive_time_gain_ * error_norm);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt * time_scale);
    last_update_time_ = current_time;

    // 重新评估期望状态
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error_new(pos_des.x() - odom_pos_.x(),
                                        pos_des.y() - odom_pos_.y());

    // 积分项更新（带抗饱和）
    if (dt > 1e-6 && error_norm < integral_limit_ * 2.0)
    {
      pos_error_integral_ += pos_error_new * dt;
      // 限制积分项幅度
      pos_error_integral_ = clampNorm(pos_error_integral_, integral_limit_);
    }

    // 微分项（速度误差）
    const Eigen::Vector2d vel_error = Eigen::Vector2d(vel_des.x(), vel_des.y()) - odom_vel_world_;

    // PID控制输出
    Eigen::Vector2d feedback_vel = kp_pos_ * pos_error_new +
                                   ki_pos_ * pos_error_integral_ +
                                   kd_pos_ * vel_error;

    // 加速度前馈（可选）
    Eigen::Vector2d acc_feedforward = Eigen::Vector2d::Zero();
    if (use_acceleration_feedforward_ && traj_.size() > 2)
    {
      const Eigen::Vector3d acc_des = traj_[2].evaluateDeBoorT(exec_time_);
      acc_feedforward = Eigen::Vector2d(acc_des.x(), acc_des.y()) * 0.1; // 动力学补偿系数
    }

    // 分层限幅：优先保证反馈控制能力
    const double max_total_vel = std::max(max_vx_, max_vy_);
    const double feedback_norm = feedback_vel.norm();
    const double feedforward_norm = Eigen::Vector2d(vel_des.x(), vel_des.y()).norm();

    Eigen::Vector2d vel_world;
    if (feedback_norm + feedforward_norm > max_total_vel)
    {
      // 按优先级分配速度预算
      const double feedback_budget = max_total_vel * feedforward_priority_;
      const double feedforward_budget = max_total_vel * (1.0 - feedforward_priority_);

      feedback_vel = clampNorm(feedback_vel, feedback_budget);
      Eigen::Vector2d vel_ff = clampNorm(Eigen::Vector2d(vel_des.x(), vel_des.y()),
                                         feedforward_budget);
      vel_world = vel_ff + feedback_vel + acc_feedforward;
    }
    else
    {
      vel_world = Eigen::Vector2d(vel_des.x(), vel_des.y()) + feedback_vel + acc_feedforward;
    }

    // 应用航向速度衰减
    vel_world *= speed_scale;

    // 转换到body坐标系并限幅
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;

    // 终止条件
    if (exec_time_ >= traj_duration_ && pos_error_new.norm() < finish_dist_)
    {
      command = geometry_msgs::msg::Twist();
      pos_error_integral_ = Eigen::Vector2d::Zero(); // 清零积分
    }

    cmd_vel_pub_->publish(command);
    last_pos_error_ = pos_error_new;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d odom_vel_world_{Eigen::Vector2d::Zero()};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

  // 控制参数
  double time_forward_, heading_error_threshold_;
  double kp_pos_, ki_pos_, kd_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;

  // 新增参数
  double integral_limit_;
  double adaptive_time_gain_;
  double heading_speed_decay_;
  bool use_acceleration_feedforward_;
  double feedforward_priority_;

  // 状态变量
  Eigen::Vector2d pos_error_integral_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_pos_error_{Eigen::Vector2d::Zero()};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
