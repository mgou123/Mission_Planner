#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_center_aligned_condition.hpp"

namespace mp_behavior_tree
{
IsCenterAlignedCondition::IsCenterAlignedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsCenterAlignedCondition::tick()
{   
  vision::DetectedObjects objects;
  vision::DetectedObject pic;
  std::string identifier;
  float center_offset_x;
  float center_offset_y;
  float x_ratio; 
  float y_ratio;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsCenterAligned] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }
  
  if (!getInput("pic_identifier", identifier)) {
    ROS_ERROR("[IsCenterAligned] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("pic_identifier", identifier);
  }

  if (!getInput("center_offset_x", center_offset_x)) {
    ROS_ERROR("[IsCenterAligned] center_offset_x not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("center_offset_x", center_offset_x);
  }

  if (!getInput("center_offset_y", center_offset_y)) {
    ROS_ERROR("[IsCenterAligned] center_offset_y not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("center_offset_y", center_offset_y);
  }

  ROS_INFO("is center aligned running");
  ROS_INFO("looking for %s", identifier.c_str());

  for (auto object : objects.detected) {
    //ROS_INFO("1");
    if (object.name.compare(identifier) == 0) {
      pic = object;
      float pic_center_x = object.centre_x;
      float pic_center_y = object.centre_y;
      float target_x = object.image_width * x_ratio;
      float target_y = object.image_height * y_ratio;
      ROS_INFO("pic_center_x %f", pic_center_x);
      ROS_INFO("pic_center_y %f", pic_center_y);
      ROS_INFO("target_x %f", target_x);
      ROS_INFO("target_y %f", target_y);
      if (abs(target_x - pic_center_x) < center_offset_x && abs(target_y - pic_center_y) < center_offset_y) {
        ROS_INFO("pic is aligned");
        return BT::NodeStatus::SUCCESS;
      } else {
        ROS_INFO("pic is not aligned");
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::FAILURE;

}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsCenterAlignedCondition>("IsCenterAligned");
}