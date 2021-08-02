#include <string>
#include <memory>

#include "auv_bt_plugins/action/calc_pic_angle.hpp"


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
  std::string target;
  float center_offset_x;
  float center_offset_y;
  float ratio;
  float x = 0;
  float y = 0;
  float yaw = 0;
  geometry_msgs::PoseStamped goal;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[CalcPicAngle] objects not provided!");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("target", target)) {
    ROS_ERROR("[CalcPicAngle] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("ratio", ratio)) {
    ROS_ERROR("[CalcPicAngle] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  }

  for (auto object : objects.detected) {
    if (object.name.compare(target) == 0) {
      pic = object;
    }
  }

  float view_center_x = pic.image_width / 2;
  float view_center_y = pic.image_height / 2;

  x = ratio * (pic.centre_x - view_center_x);
  y = - ratio * (pic.centre_y - view_center_y);

  ROS_INFO("[CalcBinAngle]: x is %f, y is %f", x, y);

  // if (abs(view_center_x - pic.centre_x) < center_offset_x) {
  //   y = ratio * (pic.centre_x - view_center_x);
  // } else if (abs(view_center_y - pic.centre_y) < center_offset_y) {
  //   x = ratio * (pic.centre_y - view_center_y);
  // } else {
  //   yaw = - atan((pic.centre_x - view_center_x) / (pic.centre_y - view_center_y)) * 180 / 6.28;
  // }

  goal.pose.position.x = x;
  goal.pose.position.y = y;

  setOutput("goal", goal);
  ROS_INFO("[CalcBinAngle] Running");

  return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::CalcPicAngle>("CalcPicAngle");
}
                                                                                                    