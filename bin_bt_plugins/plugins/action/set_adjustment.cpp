#include <string>
#include <memory>

#include "bin_bt_plugins/action/set_adjustment.hpp"

namespace mp_behavior_tree
{
  SetAdjustment::SetAdjustment(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus SetAdjustment::tick()
{   
  bb_msgs::DetectedObjects objects;
  bb_msgs::DetectedObject lid;
  float param_x; 
  float param_y;
  float x_goal;
  float y_goal;

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[SetAdjustment] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("vision_objects", objects);
  }
  
  if (!getInput("param_x", param_x)) {
    ROS_ERROR("[SetAdjustment] param_x not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("param_x", param_x);
  }

  if (!getInput("param_y", param_y)) {
    ROS_ERROR("[SetAdjustment] param_y not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("param_y", param_y);
  }

  for (auto object : objects.detected) {
    if (object.name == "Lid") {
      lid = object; 
    }
  }

  float view_center_x = lid.image_width / 2; 
  float view_center_y = lid.image_height / 2;
  
  if (view_center_x >= lid.centre_x) {
    x_goal = -param_x;
  } else {
    x_goal = param_x; 
  }
  
  if (view_center_y >= lid.centre_y) {
    y_goal = param_y;
  } else {
    y_goal = -param_y; 
  }

  setOutput("x_goal", x_goal);
  setOutput("y_goal", y_goal);

  ROS_INFO("set_adjustment running");

  return BT::NodeStatus::SUCCESS;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SetAdjustment>("SetAdjustment");
}