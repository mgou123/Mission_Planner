#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_pic_centered_condition.hpp"

namespace mp_behavior_tree
{
IsPicCenteredCondition::IsPicCenteredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsPicCenteredCondition::tick()
{   
  vision::DetectedObjects objects;
  vision::DetectedObject pic;
  std::string identifier;
  float center_offset_x; 
  float center_offset_y;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsPicCentered] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }
  
  if (!getInput("pic_identifier", identifier)) {
    ROS_ERROR("[IsPicCentered] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("pic_identifier", identifier);
  }

  if (!getInput("center_offset_x", center_offset_x)) {
    ROS_ERROR("[IsPicCentered] center_offset_x not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("center_offset_x", center_offset_x);
  }

  if (!getInput("center_offset_y", center_offset_y)) {
    ROS_ERROR("[IsPicCentered] center_offset_y not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("center_offset_y", center_offset_y);
  }

  ROS_INFO("is picture centered running");

  for (auto object : objects.detected) {
    if (object.name == identifier) {
      pic = object;
      float pic_center_x = object.centre_x;
      float pic_center_y = object.centre_y;
      float view_center_x = object.image_width / 2;
      float view_center_y = object.image_height / 2;
      ROS_INFO("pic_center_x %f", pic_center_x);
      ROS_INFO("pic_center_y %f", pic_center_y);
      ROS_INFO("view_center_x %f", view_center_x);
      ROS_INFO("view_center_y %f", view_center_y);
      if (abs(view_center_x - pic_center_x) < center_offset_x && abs(view_center_y - pic_center_y) < center_offset_y) {
        ROS_INFO("pic is centered");
        return BT::NodeStatus::SUCCESS;
      } else {
        ROS_INFO("pic is not centered");
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
  factory.registerNodeType<mp_behavior_tree::IsPicCenteredCondition>("IsPicCentered");
}