#include "closest_point_finder/closest_point_finder.hpp"
#include <motion_utils/motion_utils.hpp>

namespace closest_point_finder {

using motion_utils::findNearestIndex;

ClosestPointFinder::ClosestPointFinder()
: Node("closest_point_finder"),
  odometry_(nullptr),  // 初期化
  trajectory_(nullptr) // 初期化
{
  pub_closest_point_ = create_publisher<TrajectoryPoint>("control/debug/closest_point", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg) { 
      odometry_ = msg; 
    });
    
  sub_trajectory_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", 1, [this](const Trajectory::SharedPtr msg) { 
      trajectory_ = msg; 
    });

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&ClosestPointFinder::onTimer, this));
}

void ClosestPointFinder::onTimer()
{
  // odometry_またはtrajectory_がnullptrか確認
  if (!odometry_ || !trajectory_) {
    RCLCPP_WARN(this->get_logger(), "Odometry or trajectory is not available.");
    return;  // どちらかがnullptrの場合は処理を中断
  }

  // find closest point
  size_t closest_traj_point_idx = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish closest point
  pub_closest_point_->publish(trajectory_->points.at(closest_traj_point_idx));
}

}  // namespace closest_point_finder

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<closest_point_finder::ClosestPointFinder>());
  rclcpp::shutdown();
  return 0;
}