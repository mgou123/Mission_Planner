#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_handle_close_enough_condition.hpp"

namespace mp_behavior_tree
{
IsHandleCloseEnoughCondition::IsHandleCloseEnoughCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsHandleCloseEnoughCondition::tick()
{   
  bb_msgs::DetectedObjects objects;
  float width_benchmark; 

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsHandleCloseEnough] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("CloseEnough_ratio", CloseEnough_ratio)) {
    ROS_ERROR("[IsHandleCloseEnough] CloseEnough_ratio not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("CloseEnough_ratio", objects);
  }

  ROS_INFO("Is handle CloseEnough running");

  for (auto object : objects.detected) {
    //ROS_INFO("1");
    if (object.name.compare("Handle") == 0) {
      if (object.bbox_height > width_benchmark) {
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  
  return BT::NodeStatus::FAILURE;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsHandleCloseEnoughCondition>("IsHandleCloseEnough");
}