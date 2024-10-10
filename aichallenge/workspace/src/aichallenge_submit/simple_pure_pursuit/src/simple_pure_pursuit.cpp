#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <iostream> // Added for std::cout

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 0.5)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 0.1)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0))
{
  this->declare_parameter("minimum_trj_point_size", 16);
  minimum_trj_point_size_ = this->get_parameter("minimum_trj_point_size").as_int();
  
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  // sub_objects_ = create_subscription<Float64MultiArray>(
  //   "input/objects", 1, [this](const Float64MultiArray::SharedPtr msg) { objects_ = msg; });
  sub_steering_ = create_subscription<SteeringReport>(
    "input/steering", 10,
    [this](const SteeringReport::SharedPtr msg) { current_steering_ = msg->steering_tire_angle; });
  sub_is_pitstop_= create_subscription<Int32>(
    "/aichallenge/pitstop/is_pit", 1, [this](const Int32::SharedPtr msg) { is_pitstop_ = msg; });
  sub_velocity_= create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1, [this](const VelocityReport::SharedPtr msg) { velocity_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&SimplePurePursuit::onTimer, this));

  to_goal = 0;
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  
  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());
  double target_longitudinal_vel = 0.0;  


    // get closest trajectory point from current position
    TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

    // calc longitudinal speed and acceleration
    target_longitudinal_vel = closet_traj_point.longitudinal_velocity_mps;
    double current_longitudinal_vel = odometry_->twist.twist.linear.x;
    cmd.longitudinal.speed = target_longitudinal_vel;
    double acc = closet_traj_point.lateral_velocity_mps;
    cmd.longitudinal.acceleration = acc + speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

    // calc lateral control
    //// calc lookahead distance
    double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
    //// calc center coordinate of rear wheel
    double rear_x = odometry_->pose.pose.position.x -
                    wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
    double rear_y = odometry_->pose.pose.position.y -
                    wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
    //// search lookahead point
    auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
      [&](const TrajectoryPoint & point) {
        return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
                lookahead_distance;
      });
    if (lookahead_point_itr == trajectory_->points.end()) {
      lookahead_point_itr = trajectory_->points.end() - 1;
    }
    double lookahead_point_x = lookahead_point_itr->pose.position.x;
    double lookahead_point_y = lookahead_point_itr->pose.position.y;

    geometry_msgs::msg::PointStamped lookahead_point_msg;
    lookahead_point_msg.header.stamp = get_clock()->now();
    lookahead_point_msg.header.frame_id = "map";
    lookahead_point_msg.point.x = lookahead_point_x;
    lookahead_point_msg.point.y = lookahead_point_y;
    lookahead_point_msg.point.z = 0;
    pub_lookahead_point_->publish(lookahead_point_msg);

    // calc steering angle for lateral control

  

    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                    tf2::getYaw(odometry_->pose.pose.orientation) + 5 * M_PI / 180.0;
    
    // std::cout << "dy: " << lookahead_point_y - rear_y << " dx: " << lookahead_point_x - rear_x
    //           << " atan2: " << std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) 
    //           << " yaw: " << tf2::getYaw(odometry_->pose.pose.orientation)
    //           << " alpha: " << alpha << std::endl;
    cmd.lateral.steering_tire_angle =2 * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

  
  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "Odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "Trajectory is not available");
    return false;
  }
  // if (!objects_) {
  //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "Objects are not available");
  //   return false;
  // }
  if (!velocity_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "Velocity is  not available");
    return false;
  }
  return true;
}

}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}