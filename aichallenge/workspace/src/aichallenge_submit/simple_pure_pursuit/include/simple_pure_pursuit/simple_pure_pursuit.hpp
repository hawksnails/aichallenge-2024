#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include<math.h>

namespace simple_pure_pursuit {
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64MultiArray;
using std_msgs::msg::Int32;
class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_objects_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;
  Float64MultiArray::SharedPtr objects_;
  Int32::SharedPtr is_pitstop_;
  VelocityReport::SharedPtr velocity_;

  double current_steering_;
  double current_velocity_;

  // pure pursuit parameters
  const double wheel_base_;
  const double lookahead_gain_;
  const double lookahead_min_distance_;
  const double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  long unsigned int minimum_trj_point_size_;
  unsigned int to_goal; // 0 for goal, 1 fo half goal, 2 for pitstop
 private:
  void onTimer();
  bool subscribeMessageAvailable();
  bool is_decelerated_pitstop;
  bool is_start;
};
}  // namespace simple_pure_pursuit
#endif  // SIMPLE_PURE_PURSUIT_HPP_