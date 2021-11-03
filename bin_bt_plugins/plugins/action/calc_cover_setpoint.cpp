#include <string>
#include <memory>

#include "bin_bt_plugins/action/calc_cover_setpoint.hpp"

namespace mp_behavior_tree
{

CalcCoverSetpoint::CalcCoverSetpoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus CalcCoverSetpoint::CalcCoverSetpoint::tick()
{   
  std::vector<bb_msgs::DetectedObject> objects;
  bb_msgs::DetectedObject lid_1;
  bb_msgs::DetectedObject lid_2;
  int pre_direction; 
  float lift_ratio_x;
  float lift_ratio_y;
  float ratio;
  float drop_pose_1; 
  float lift_pose_1_x;
  float lift_pose_1_y; 
  int count = 0; 

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[CalcCoverSetpoint] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("lift_ratio_x", lift_ratio_x)) {
    ROS_ERROR("[CalcCoverSetpoint] lift_ratio_x not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("lift_ratio_x", lift_ratio_x);
  }

  if (!getInput("lift_ratio_y", lift_ratio_y)) {
    ROS_ERROR("[CalcCoverSetpoint] lift_ratio_y not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("lift_ratio_y", lift_ratio_y);
  }

  if (!getInput("ratio", ratio)) {
    ROS_ERROR("[CalcCoverSetpoint] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("ratio", ratio);
  }

  if (!getInput("pre_direction", pre_direction)) {
    ROS_ERROR("[CalcCoverSetpoint] pre_direction not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("pre_direction", pre_direction);
  }

  for (auto object : objects) {
    if (object.name == "Lid" && count == 0) {
      lid_1 = object; 
      count++;
    } else if (object.name == "Lid" && count == 1) {
      lid_2 = object;
      count++;
    }
  }

  float image_centre_x = lid_1.image_width / 2;
  float image_centre_y = lid_1.image_height / 2;
  
  lift_pose_1_x = (lid_1.centre_x - lid_1.bbox_width / 2 + lift_ratio_x * lid_1.bbox_width - image_centre_x) * ratio; 
  lift_pose_1_y = - (lid_1.centre_y - lid_1.bbox_height / 2 + lift_ratio_y * lid_1.bbox_height - image_centre_y) * ratio; 

  int drop_forward = 1; 
  
  if (count == 0) {
    ROS_ERROR("Calaulating cover setpoint, Not enough lids seen");
  } else if (count == 1) {
    if (pre_direction == 0) {
      ROS_ERROR("Calsulating 2nd cover setpoint. Not enough lids seen");
      return BT::NodeStatus::FAILURE;
    } else {
      drop_forward = - pre_direction / abs(pre_direction);
    }
  } else {
    if (lid_1.centre_y > lid_2.centre_y) {
      drop_forward = 1;
    } else {
      drop_forward = -1; 
    }
  }

  drop_pose_1 = drop_forward * 2; 
  
  ROS_INFO("lifting at forward %f, sideways %f, dropping at forward %f", lift_pose_1_y, lift_pose_1_x, drop_pose_1);

  setOutput("drop_pose_1", drop_pose_1);
  setOutput("lift_pose_1_x", lift_pose_1_x);
  setOutput("lift_pose_1_y", lift_pose_1_y);

  ROS_INFO("calc_cover_setpoint running");

  return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::CalcCoverSetpoint>("CalcCoverSetpoint");
}