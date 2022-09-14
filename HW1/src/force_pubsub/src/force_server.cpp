#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_forces.hpp"
#include "../include/Utils.hpp"

#include <memory>

void add(const std::shared_ptr<tutorial_interfaces::srv::AddTwoForces::Request> request,
          std::shared_ptr<tutorial_interfaces::srv::AddTwoForces::Response> response)
{
  response->result = request->f1 + request->f2;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Incoming request\nf1:"<< request->f1 << " f2:" << request->f2);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "sending back response:"<<  response->result);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_forces_server");

  rclcpp::Service<tutorial_interfaces::srv::AddTwoForces>::SharedPtr service =
    node->create_service<tutorial_interfaces::srv::AddTwoForces>("add_two_forces", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two forces.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
