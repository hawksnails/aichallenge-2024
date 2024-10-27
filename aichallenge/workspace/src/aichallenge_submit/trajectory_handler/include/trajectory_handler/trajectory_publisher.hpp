#pragma once

#include "rclcpp/rclcpp.hpp"
#include "trajectory_handler/srv/path_info.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

class TrajectoryHandler : public rclcpp::Node
{
public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  TrajectoryHandler();
private:
  void pub_trajectory(const std::shared_ptr<trajectory_handler::srv::PathInfo::Request> request,
                      std::shared_ptr<trajectory_handler::srv::PathInfo::Response> response);
  rclcpp::Service<trajectory_handler::srv::PathInfo>::SharedPtr service_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
};