// File: add_two_ints_server.cpp
#include "rclcpp/rclcpp.hpp"
#include "trajectory_handler/srv/path_info.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <memory>

class TrajectoryHandler : public rclcpp::Node
{
public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  TrajectoryHandler()
  : Node("trajectory_handler")
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    service_ = this->create_service<trajectory_handler::srv::PathInfo>("add_two_ints", std::bind(&TrajectoryHandler::handle_service, this, _1, _2));
    pub_ = this->create_publisher<Trajectory>("output", 1);
  }
private:
  void handle_service(const std::shared_ptr<trajectory_handler::srv::PathInfo::Request> request,
                      std::shared_ptr<trajectory_handler::srv::PathInfo::Response> response)
  {
    // response->sum = request->a + request->b;
    response->error_code = 0;

    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    // trajectory.header = msg->header;  // 元のメッセージのヘッダーを使用
    trajectory.header.stamp = this->now();
    trajectory.header.frame_id = "map";

    for (int i = 0; i < 10; i++) {
        autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
        trajectory_point.pose.position.x = 0;
        trajectory_point.pose.position.y = 0;
        auto quat = Eigen::Quaterniond::Identity();
        quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        trajectory_point.pose.orientation.x = quat.x();
        trajectory_point.pose.orientation.y = quat.y();
        trajectory_point.pose.orientation.z = quat.z();
        trajectory_point.pose.orientation.w = quat.w();
        trajectory_point.longitudinal_velocity_mps = 0;
        trajectory_point.lateral_velocity_mps = 0; // 横速度は使わないので加速度を入れる
        trajectory_point.heading_rate_rps = 0;
        
        trajectory.points.emplace_back(std::move(trajectory_point));
    }

    // Trajectory メッセージをパブリッシュ
    pub_->publish(trajectory);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\npath: %s", request->csv_path.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
  }
  rclcpp::Service<trajectory_handler::srv::PathInfo>::SharedPtr service_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryHandler>());
  rclcpp::shutdown();
  return 0;
}