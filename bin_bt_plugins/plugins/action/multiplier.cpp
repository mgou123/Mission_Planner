#include <string>
#include <memory>

#include "bin_bt_plugins/action/multiplier.hpp"

namespace mp_behavior_tree
{
  Multiplier::Multiplier(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus Multiplier::tick()
{   
  float orig_value;
  float value_mul;
  float result; 

  if (!getInput("orig_value", orig_value)) {
      ROS_ERROR("[Multiplier] orig_value not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("orig_value", orig_value);
  }
  
  if (!getInput("value_mul", value_mul)) {
    ROS_ERROR("[Multiplier] value_multiplied not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("value_mul", value_mul);
  }

  result = orig_value * value_mul; 

  setOutput("result", result);

  ROS_INFO("Multiplier running");

  return BT::NodeStatus::SUCCESS;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::Multiplier>("Multiplier");
}