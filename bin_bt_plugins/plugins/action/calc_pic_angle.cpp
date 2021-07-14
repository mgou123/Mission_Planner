#include <string>
#include <memory>

#include "bin_bt_plugins/action/calc_pic_angle.hpp"


namespace mp_behavior_tree
{

CalcPicAngle::CalcPicAngle(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus CalcPicAngle::CalcPicAngle::tick()
{   
  vision::DetectedObjects objects;
  vision::DetectedObject pic;
  std::string identifier;
  float center_offset_x; 
  float center_offset_y; 
  float ratio;
  float x;
  float y;
  float yaw;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[CalcPicAngle] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("pic_identifier", identifier)) {
    ROS_ERROR("[CalcPicAngle] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("pic_identifier", identifier);
  }

  if (!getInput("ratio", ratio)) {
    ROS_ERROR("[CalcPicAngle] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("ratio", ratio);
  }

  if (!getInput("center_offset_x", center_offset_x)) {
    ROS_ERROR("[CalcPicAngle] center_offset_x not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("center_offset_x", center_offset_x);
  }

  if (!getInput("center_offset_y", center_offset_y)) {
    ROS_ERROR("[CalcPicAngle] center_offset_y not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("center_offset_y", center_offset_y);
  }

  for (auto object : objects.detected) {
    if (object.name == identifier) {
      pic = object;
    }
  }
  
  float view_center_x = pic.image_width / 2;
  float view_center_y = pic.image_height / 2;
  
  if (abs(view_center_x - pic.centre_x) < center_offset_x) {
    y = ratio * (pic.centre_x - view_center_x);
  } else if (abs(view_center_y - pic.centre_y) < center_offset_y) {
    x = ratio * (pic.centre_y - view_center_x);
  } else {
    yaw = atan2(pic.centre_x - view_center_y, pic.centre_x - view_center_x);
  }

  setOutput("x_goal", x);
  setOutput("y_goal", y);
  setOutput("yaw_goal", yaw);

  return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::CalcPicAngle>("CalcPicAngle");
}