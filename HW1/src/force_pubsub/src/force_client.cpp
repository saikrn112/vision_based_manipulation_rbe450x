#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_forces.hpp"
#include "../include/Utils.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("force_client");
  rclcpp::Client<tutorial_interfaces::srv::AddTwoForces>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::AddTwoForces>("add_two_forces");

  auto request = std::make_shared<tutorial_interfaces::srv::AddTwoForces::Request>();
  request->f1 = create_vector3(1,2,3);
  request->f2 = create_vector3(3,4,5);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sum:" << result.get()->result);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service force_client");
  }

  rclcpp::shutdown();
  return 0;
}
