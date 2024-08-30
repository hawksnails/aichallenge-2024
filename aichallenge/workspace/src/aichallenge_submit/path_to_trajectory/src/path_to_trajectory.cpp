// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "path_to_trajectory/path_to_trajectory.hpp"

PathToTrajectory::PathToTrajectory() : Node("path_to_trajectory_node") {
  using std::placeholders::_1;

  pub_ = this->create_publisher<Trajectory>("output", 1);
  sub_ = this->create_subscription<PathWithLaneId>(
      "input", 1, std::bind(&PathToTrajectory::callback, this, _1));
  sub_objects_ = this->create_subscription<MarkerArray>(
    "/aichallenge/objects_marker", 1, [this](const MarkerArray::SharedPtr msg) { objects_ = msg; });
}

void PathToTrajectory::callback(const PathWithLaneId::SharedPtr msg) {
  Trajectory trajectory;
  trajectory.header = msg->header;
  for (auto& path_point_with_lane_id : msg->points) {
    TrajectoryPoint trajectory_point;
    trajectory_point.pose = path_point_with_lane_id.point.pose;
    trajectory_point.longitudinal_velocity_mps = path_point_with_lane_id.point.longitudinal_velocity_mps;
    trajectory.points.emplace_back(std::move(trajectory_point));
  }
  avoidObstacles(trajectory);
  pub_->publish(trajectory);
}

void PathToTrajectory::avoidObstacles(Trajectory &trajectory) {
   // Check if objects are available
  if (!objects_) {
    return;
  }

  std::cout << "avoidObstacles: " << sizeof(trajectory.points) << std::endl;

  constexpr double SAFETY_DISTANCE = 4.0; // Increased safety distance in meters
  constexpr double MAX_STEERING_OFFSET = 3.4; // Maximum steering offset in meters

  bool is_avoid = false;

  // Iterate through the trajectory points
  for (auto & traj_point : trajectory.points) {
    for (const auto & marker : objects_->markers) {
      double obj_x = marker.pose.position.x;
      double obj_y = marker.pose.position.y;
      double car_x = traj_point.pose.position.x;
      double car_y = traj_point.pose.position.y;

      // Check if the object is close to the trajectory point
      double distance = std::hypot(car_x - obj_x, car_y - obj_y);
      if (distance < SAFETY_DISTANCE) {
        // Apply a larger steering offset based on proximity to the obstacle
        double steering_offset = MAX_STEERING_OFFSET * (SAFETY_DISTANCE - distance) / SAFETY_DISTANCE;
        
        // Determine direction to steer
        double direction = (obj_y > car_y) ? -1.0 : 1.0; // Steer right if object is on the left, otherwise steer left

        // Apply the steering offset
        traj_point.pose.position.y += direction * steering_offset;

        // Adjust the x position to ensure smooth transition
        traj_point.pose.position.x += (obj_x > car_x) ? -steering_offset / 2 : steering_offset / 2;

        // Adjust velocity and other parameters for smoother transition
        traj_point.longitudinal_velocity_mps *= (1.0 - distance / SAFETY_DISTANCE);
        traj_point.lateral_velocity_mps = direction * steering_offset;

        // Adjust acceleration to be higher when closer to the obstacle
        traj_point.acceleration_mps2 = (distance < 1.0) ? MAX_STEERING_OFFSET : MAX_STEERING_OFFSET / distance;

        traj_point.heading_rate_rps = direction * std::atan2(traj_point.pose.position.y - car_y, traj_point.pose.position.x - car_x);
        traj_point.front_wheel_angle_rad = direction * steering_offset / 2;
        traj_point.rear_wheel_angle_rad = -direction * steering_offset / 2;

        // Set time_from_start for smoother transition
        traj_point.time_from_start.sec = static_cast<int32_t>(distance / traj_point.longitudinal_velocity_mps);
        traj_point.time_from_start.nanosec = static_cast<uint32_t>((distance / traj_point.longitudinal_velocity_mps - traj_point.time_from_start.sec) * 1e9);

        is_avoid = true;
      }
    }
  }
  if(is_avoid) {
    smoothTrajectory(trajectory);
  }
}

void PathToTrajectory::smoothTrajectory(Trajectory &trajectory) {
  if (trajectory.points.size() < 5) {
      // Not enough points to smooth
      return;
  }
  Trajectory smoothed_trajectory = trajectory;
  int window_size = 7; // Number of points to consider for moving average
  for (size_t i = window_size / 2; i < trajectory.points.size() - window_size / 2; ++i) {
      double sum_x = 0.0;
      double sum_y = 0.0;
      for (int j = -window_size / 2; j <= window_size / 2; ++j) {
          sum_x += trajectory.points[i + j].pose.position.x;
          sum_y += trajectory.points[i + j].pose.position.y;
      }
      smoothed_trajectory.points[i].pose.position.x = sum_x / window_size;
      smoothed_trajectory.points[i].pose.position.y = sum_y / window_size;
  }
    // Directly assign the points to the trajectory
  trajectory.points = smoothed_trajectory.points;
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
