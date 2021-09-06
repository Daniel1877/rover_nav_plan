#ifndef ROVER_PLANNING_BEHAVIOR_TREE_NODES__PHOTO_HPP_
#define ROVER_PLANNING_BEHAVIOR_TREE_NODES__PHOTO_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace plansys2_bt_example
{

using namespace std::chrono_literals;


class Photo : public BT::ActionNodeBase
{
public:
  explicit Photo(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    //return BT::PortsList({});
    return {
      BT::InputPort<std::string>("camera")
    };
  }

private:
  int counter_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client;
  rclcpp::Node::SharedPtr node;

  //ImageSaver &image_saver;
};

}  // namespace rover_planning

#endif  // ROVER_PLANNING_BEHAVIOR_TREE_NODES__PHOTO_HPP_
