#include <string>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <cstring>

#include "rover_planning/behavior_tree_nodes/Photo.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"


namespace plansys2_bt_example
{

Photo::Photo(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
    config().blackboard->get("node", node);
}

void
Photo::halt()
{
  std::cout << "Photo halt" << std::endl;
}

BT::NodeStatus
Photo::tick()
{

  std::cout << "Photo tick " << counter_ << std::endl;

  std::string camera;
  getInput<std::string>("camera", camera);

  std::cout << "/"+camera+"/save" << std::endl;



  if (counter_++ < 10) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    
    client = node->create_client<std_srvs::srv::Empty>("/"+camera+"/save");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crea cliente.");

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return BT::NodeStatus::FAILURE;
      }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crea la solicitud.");

    auto result = client->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Envía el resultado.");

    return BT::NodeStatus::SUCCESS;
  }

}

}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_bt_example::Photo>("Photo");
}
