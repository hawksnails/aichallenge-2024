#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include "trajectory_handler/srv/path_info.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <chrono>
using namespace std::chrono_literals;

class TrajectoryHandler : public rclcpp::Node
{
public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using Odometry = nav_msgs::msg::Odometry;
  TrajectoryHandler() : Node("trajectory_handler")
  {
    client = this->create_client<trajectory_handler::srv::PathInfo>("/planning/scenario_planning/path_info");
    sub_odom_ = create_subscription<Odometry>("input/kinematics", 1, std::bind(&TrajectoryHandler::callback, this, std::placeholders::_1));
    sub_traj_ = create_subscription<Trajectory>("input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  }
private:
    Trajectory::SharedPtr trajectory_;
    std::vector<std::string> file_name_list = {
        "1.csv",
        "2.csv",
        "3.csv",
    };

    int send_request(std::string filename){
        auto request = std::make_shared<trajectory_handler::srv::PathInfo::Request>();
        request->csv_path = filename;
        auto result = client->async_send_request(request);
        return result.get()->error_code;
    }

    void callback(const Odometry::SharedPtr msg){
        static int path_index = 0;
        if (trajectory_ == nullptr) {
            send_request(file_name_list.at(0));
        }
        auto publish_flg_point = trajectory_->points.at(trajectory_->points.size() - 100);
        double distance = std::hypot(publish_flg_point.pose.position.x - msg->pose.pose.position.x, publish_flg_point.pose.position.y - msg->pose.pose.position.y);
        if (distance < 2.0) {
            path_index++;
            if (path_index >= file_name_list.size()) {
                path_index = 0;
            }
            int ret = send_request(file_name_list.at(path_index));
            if (ret != 0) {
                path_index--;
            }
        }

    }
    rclcpp::Client<trajectory_handler::srv::PathInfo>::SharedPtr client;
    rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryHandler>());
    rclcpp::shutdown();

    return 0;
}



