#include "closest_point_finder/closest_point_finder.hpp"

#include <motion_utils/motion_utils.hpp>

namespace  closest_point_finder
{

using motion_utils::findNearestIndex;

ClosestPointFinder::ClosestPointFinder()
: Node("closest_point_finder")
{
  pub_closest_point_ = create_publisher<TrajectoryPoint>("control/debug/closest_point", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&ClosestPointFinder::onTimer, this));
}

void ClosestPointFinder::onTimer()
{
  // find closest point
  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish closest point
  pub_closest_point_->publish(trajectory_->points.at(closet_traj_point_idx));

}

}  // namespace closest_point_finder

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<closest_point_finder::ClosestPointFinder>());
  rclcpp::shutdown();
  return 0;
}
