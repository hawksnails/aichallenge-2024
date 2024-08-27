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
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_gear_ = create_publisher<GearCommand>("output/gear_cmd", 1);
  
  
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  sub_objects_ = create_subscription<Float64MultiArray>(
    "input/objects", 1, [this](const Float64MultiArray::SharedPtr msg) { objects_ = msg; });
  sub_steering_ = create_subscription<SteeringReport>(
    "input/steering", 10,
    [this](const SteeringReport::SharedPtr msg) { current_steering_ = msg->steering_tire_angle; });
  sub_velocity_ = create_subscription<VelocityReport>(
    "input/velocity", 10,
    [this](const VelocityReport::SharedPtr msg) { current_velocity_ = msg->longitudinal_velocity; });
  
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&SimplePurePursuit::onTimer, this));
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

GearCommand zeroGearCommand(rclcpp::Time stamp)
{
  GearCommand gear_cmd;
  gear_cmd.stamp = stamp;
  gear_cmd.command = 2;
  return gear_cmd;
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
  GearCommand gear_cmd = zeroGearCommand(get_clock()->now());
  
  if (
    (closet_traj_point_idx == trajectory_->points.size() - 1) ||
    (trajectory_->points.size() <= 5)) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
   }else {
    // get closest trajectory point from current position
    TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);
    
    // calculate longitudinal speed and acceleration
    double target_longitudinal_vel = use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
    double current_longitudinal_vel = odometry_->twist.twist.linear.x;
    cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration = speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

    // calculate lookahead distance
    double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;

    // calculate center coordinate of rear wheel
    double rear_x = odometry_->pose.pose.position.x - wheel_base_ / 2.0 * std::cos(tf2::getYaw(odometry_->pose.pose.orientation));
    double rear_y = odometry_->pose.pose.position.y - wheel_base_ / 2.0 * std::sin(tf2::getYaw(odometry_->pose.pose.orientation));

    // search lookahead point
    auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
      [&](const TrajectoryPoint & point) {
        return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >= lookahead_distance;
      });
      
    if (lookahead_point_itr == trajectory_->points.end()) {
      lookahead_point_itr = trajectory_->points.end() - 1;
    }

    double lookahead_point_x = lookahead_point_itr->pose.position.x;
    double lookahead_point_y = lookahead_point_itr->pose.position.y;

    // calculate steering angle for lateral control
    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) - tf2::getYaw(odometry_->pose.pose.orientation);
    cmd.lateral.steering_tire_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);


    // Object avoidance
    for (size_t i = 0; i < objects_->data.size(); i += 4) {
      double object_x = objects_->data[i];
      double object_y = objects_->data[i + 1];
      double object_radius = objects_->data[i + 3];
      double object_distance = std::hypot(object_x - odometry_->pose.pose.position.x, object_y - odometry_->pose.pose.position.y);
      double object_angle = std::atan2(object_y - odometry_->pose.pose.position.y, object_x - odometry_->pose.pose.position.x);

      double object_radius_sum = object_radius + 4.5;
      //車体から見た物体の角度
      double object_angle_diff = object_angle - tf2::getYaw(odometry_->pose.pose.orientation);
      
      if (current_steering_ < 1.0 && current_steering_ > -1.0){
      //障害物が近づいたとき回避する
      if (object_distance < object_radius_sum && object_distance > object_radius+0.4){
        //物体が前方にあるとき
        if ((object_angle_diff <0.5&&  object_angle_diff > 0.1) ||  (object_angle_diff < -0.1 && object_angle_diff > -0.5)){
          //物体が右側にあるとき
          if (object_angle_diff < 0){
            //左側に避ける
            if (current_velocity_ > 7){
              cmd.lateral.steering_tire_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance) + 0.4;
            } else {
              cmd.lateral.steering_tire_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance) + 0.6;
            }
          } else {
            //右側に避ける
            if (current_velocity_ > 7){
              cmd.lateral.steering_tire_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance) - 0.4;
            } else {
              cmd.lateral.steering_tire_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance) - 0.6;
            }
          }
        }
      }
      }

      //速度が0のとき，壁にぶつかっているので，ステアを切る
      if (current_velocity_ < 0.3){
        //現在の座標を見る
        double current_x = odometry_->pose.pose.position.x;
        double current_y = odometry_->pose.pose.position.y;
        gear_cmd.command = 20;
        //今の位置から1m後ろに戻る
        while (std::hypot(current_x - odometry_->pose.pose.position.x, current_y - odometry_->pose.pose.position.y) < 2.0){
          cmd.longitudinal.acceleration = -3.0;
        }
        //通常の動作に戻す
        gear_cmd.command = 2;
      }
      std::cout << "Object x: " << object_x << " Object y: " << object_y << std::endl;
    }
  }
  pub_cmd_->publish(cmd);
  pub_gear_->publish(gear_cmd);
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
  if (!objects_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "Objects are not available");
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