#include <string>
#include <memory>

#include "bin_bt_plugins/action/adder.hpp"

namespace mp_behavior_tree
{
  Adder::Adder(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus Adder::tick()
{   
  float orig_value;
  float value_added;
  float result; 

  if (!getInput("orig_value", orig_value)) {
      ROS_ERROR("[Adder] orig_value not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("orig_value", orig_value);
  }
  
  if (!getInput("value_added", value_added)) {
    ROS_ERROR("[Adder] value_added not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("value_added", value_added);
  }

  result = orig_value + value_added; 

  setOutput("result", result);

  ROS_INFO("Adder running");

  return BT::NodeStatus::SUCCESS;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::Adder>("Adder");
}