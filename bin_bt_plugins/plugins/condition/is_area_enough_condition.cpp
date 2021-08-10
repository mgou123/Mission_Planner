#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_area_enough_condition.hpp"

namespace mp_behavior_tree
{
IsAreaEnoughCondition::IsAreaEnoughCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsAreaEnoughCondition::tick()
{   
  vision::DetectedObjects objects;
  std::string identifier;
  float area = 0;
  bool passed = true; 
  bool detected = false; 

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsAreaEnough] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }
  
  if (!getInput("pic_identifier", identifier)) {
    ROS_ERROR("[IsAreaEnough] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("pic_identifier", identifier);
  }

  if (!getInput("area", area)) {
    ROS_ERROR("[IsAreaEnough] area not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("area", area);
  }

  // ROS_INFO("is picture centered running");
  ROS_INFO("checking area of %s", identifier.c_str());

  for (auto object : objects.detected) {
    //ROS_INFO("1");
    detected = true;
    if (object.name.compare(identifier) == 0) {
      if (object.bbox_width * object.bbox_height < area) {
        passed = passed && false; 
      } else {
        passed = passed && true;
      }
    }
  }

  if (passed && detected) {
    ROS_INFO("Area is enough");
    return BT::NodeStatus::SUCCESS;
  } 

  return BT::NodeStatus::FAILURE;

}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsAreaEnoughCondition>("IsAreaEnough");
}