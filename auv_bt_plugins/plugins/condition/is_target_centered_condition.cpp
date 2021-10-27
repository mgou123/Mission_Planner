#include <string>
#include <memory>

#include "auv_bt_plugins/condition/is_target_centered_condition.hpp"

namespace mp_behavior_tree
{
IsTargetCenteredCondition::IsTargetCenteredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsTargetCenteredCondition::tick()
{
  std::vector<bb_msgs::DetectedObject> objects;
  bb_msgs::DetectedObject pic;
  std::string identifier;
  float center_offset_x;
  float center_offset_y;
  float x_ratio;
  float y_ratio;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsCenterAligned] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else if (!getInput("pic_identifier", identifier)) {
    ROS_ERROR("[IsCenterAligned] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else if (!getInput("center_offset_x", center_offset_x)) {
    ROS_ERROR("[IsCenterAligned] center_offset_x not provided!");
    return BT::NodeStatus::FAILURE;
  } else if (!getInput("center_offset_y", center_offset_y)) {
    ROS_ERROR("[IsCenterAligned] center_offset_y not provided!");
    return BT::NodeStatus::FAILURE;
  }

  for (auto object : objects) {
    if (object.name.compare(identifier) == 0) {
      pic = object;
    }
  }

  float view_center_x = pic.image_width / 2;
  float view_center_y = pic.image_height / 2;
  float target_center_x = pic.centre_x;
  float target_center_y = pic.centre_y;

  if(abs(target_center_x - view_center_x) < center_offset_x && abs(target_center_y - view_center_y) < center_offset_y) {
    ROS_INFO("[IsTargetCentered]: Target is aligned");
    return BT::NodeStatus::SUCCESS;
  }
  ROS_INFO("[IsTargetCentered]: Target is NOT aligned");
  return BT::NodeStatus::FAILURE;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsTargetCenteredCondition>("IsCenterAligned");
}          