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
    sub_odom_ = create_subscription<Odometry>("/localization/kinematic_state", 1, std::bind(&TrajectoryHandler::callback, this, std::placeholders::_1));
    sub_traj_ = create_subscription<Trajectory>("input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  }
private:
    Trajectory::SharedPtr trajectory_;
    std::vector<std::string> file_name_list = {
        "1.csv"
        // "2.csv",
        // "3.csv",
    };

    double last_update_time = 0;

    void send_request_async(std::string filename) {
        auto request = std::make_shared<trajectory_handler::srv::PathInfo::Request>();
        request->csv_path = filename;

        using ServiceResponseFuture = rclcpp::Client<trajectory_handler::srv::PathInfo>::SharedFuture;
        auto response_received_callback = [this, filename](ServiceResponseFuture future) {
            auto result = future.get();
            if (result->error_code != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service for %s", filename.c_str());
                path_index--;  // エラーの場合はインデックスを戻す
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully published %s", filename.c_str());
                this->last_update_time = now().seconds();
            }
        };

        client->async_send_request(request, response_received_callback);
    }
    int path_index = 0;
    void callback(const Odometry::SharedPtr msg){
        if (trajectory_ == nullptr) {
            RCLCPP_INFO(get_logger(), "initial trajectory");
            send_request_async(file_name_list.at(0));
            return;
        }
        if (now().seconds() - this->last_update_time < 3) {
            // 前回の更新から3秒以内は更新しない
            return;
        }
        double last_point_t = trajectory_->points.at(trajectory_->points.size() - 1).time_from_start.sec;
        int flg_index = 0;
        for (int i = trajectory_->points.size() - 1; i >= 0; i--) {
            if (last_point_t - trajectory_->points.at(i).time_from_start.sec > 3) {
                break;
            }
            flg_index = i;
        }
        auto publish_flg_point = trajectory_->points.at(flg_index);
        double distance = std::hypot(publish_flg_point.pose.position.x - msg->pose.pose.position.x, publish_flg_point.pose.position.y - msg->pose.pose.position.y);
        // RCLCPP_INFO(get_logger(), "distance: %f", distance);
        if (distance < 2.0) {
            path_index++;
            if (path_index >= file_name_list.size()) {
                path_index = 0;
            }
            send_request_async(file_name_list.at(path_index));
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



