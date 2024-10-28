#include "rclcpp/rclcpp.hpp"
#include "trajectory_handler/srv/path_info.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("path_setter");

    rclcpp::Client<trajectory_handler::srv::PathInfo>::SharedPtr client =
        node->create_client<trajectory_handler::srv::PathInfo>("/planning/scenario_planning/path_info");

    auto request = std::make_shared<trajectory_handler::srv::PathInfo::Request>();

    if (argc < 2) {
        RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run trajectory_handler path_setter <csv_path>");
        return 1;
    }

    request->csv_path = argv[1];

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "error_code: %d", result.get()->error_code);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    }

    rclcpp::shutdown();

    return 0;
}