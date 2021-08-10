#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_handle_flat_condition.hpp"

namespace mp_behavior_tree
{
IsHandleFlatCondition::IsHandleFlatCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsHandleFlatCondition::tick()
{   
  vision::DetectedObjects objects;
  vision::DetectedObject handle_1;
  vision::DetectedObject handle_2;
  float flat_ratio; 
  int count = 0;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsHandleFlat] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("flat_ratio", flat_ratio)) {
    ROS_ERROR("[IsHandleFlat] flat_ratio not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("flat_ratio", objects);
  }

  ROS_INFO("Is handle flat running");

  for (auto object : objects.detected) {
    //ROS_INFO("1");
    if (object.name.compare("Handle") == 0) {
      if (count == 0) {
        count ++;
        handle_1 = object; 
      } else if (count == 1) {
        count ++;
        handle_2 = object;
      }
    }
  }
  
  if (count < 2) {
    ROS_INFO("not enough handles seen");
    return BT::NodeStatus::FAILURE;
  }

  if (handle_1.bbox_height / handle_1.bbox_width < flat_ratio && handle_2.bbox_height / handle_2.bbox_width < flat_ratio) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsHandleFlatCondition>("IsHandleFlat");
}