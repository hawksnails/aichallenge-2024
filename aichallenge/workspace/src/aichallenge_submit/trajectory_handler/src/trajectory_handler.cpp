#include "trajectory_handler/trajectory_handler.hpp"
#include "trajectory_handler/trajectory_generator.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <memory>


TrajectoryHandler::TrajectoryHandler() : Node("trajectory_handler_node"){
    using std::placeholders::_1;
    using std::placeholders::_2;
    this->service_ = this->create_service<trajectory_handler::srv::PathInfo>("add_two_ints", std::bind(&TrajectoryHandler::pub_trajectory, this, _1, _2));
    this->pub_ = this->create_publisher<Trajectory>("output", 1);
}

void TrajectoryHandler::pub_trajectory(const std::shared_ptr<trajectory_handler::srv::PathInfo::Request> request,
                      std::shared_ptr<trajectory_handler::srv::PathInfo::Response> response)
{

    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    // trajectory.header = msg->header;  // 元のメッセージのヘッダーを使用

    TrajectoryGenerator traj;
    try {
        std::string dir_path = "/aichallenge/workspace/src/aichallenge_submit/path_to_trajectory/src/";
        traj = TrajectoryGenerator(dir_path + request->csv_path);
    } catch (const std::exception& e) {
        std::cout << "in constructor of PathTrajectory:" << e.what() << std::endl;
        response->error_code = 1;
        return;
    }
    std::vector<PathPoint> generated_path = traj.generate_path(0.1);

    trajectory.header.stamp = this->now();
    trajectory.header.frame_id = "map";

    for (const auto& path_point : generated_path) {
        autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
        trajectory_point.pose.position.x = path_point.x;
        trajectory_point.pose.position.y = path_point.y;
        auto quat = Eigen::Quaterniond::Identity();
        quat = Eigen::AngleAxisd(path_point.th, Eigen::Vector3d::UnitZ());
        trajectory_point.pose.orientation.x = quat.x();
        trajectory_point.pose.orientation.y = quat.y();
        trajectory_point.pose.orientation.z = quat.z();
        trajectory_point.pose.orientation.w = quat.w();
        trajectory_point.longitudinal_velocity_mps = path_point.vel;
        // trajectory_point.lateral_velocity_mps = path_point.acc; // 横速度は使わないので加速度を入れる
        // trajectory_point.heading_rate_rps = path_point.curvature;
        
        trajectory.points.emplace_back(std::move(trajectory_point));
    }

    // Trajectory メッセージをパブリッシュ
    pub_->publish(trajectory);

    response->error_code = 0;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\npath: %s", request->csv_path.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryHandler>());
  rclcpp::shutdown();
  return 0;
}