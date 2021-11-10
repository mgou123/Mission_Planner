#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_full_bin_seen_condition.hpp"

namespace mp_behavior_tree
{

IsFullBinSeenCondition::IsFullBinSeenCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsFullBinSeenCondition::tick()
{   
  std::vector<bb_msgs::DetectedObject> objects; 
  float area_benchmark;
  float x_min = 10000; 
  float x_max = 0; 
  float y_min = 10000; 
  float y_max = 0; 
  float area = -1;

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[FullBinSeen] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("vision_objects", objects);
  }
  
  if (!getInput("area_benchmark", area_benchmark)) {
      ROS_ERROR("[FullBinSeen] area_benchmark not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("area_benchmark", area_benchmark);
  }

  for (auto object : objects) {
      if (object.centre_x - object.bbox_width / 2 < x_min) {
        x_min = object.centre_x - object.bbox_width / 2;
      } 
      if (object.centre_x + object.bbox_width / 2 > x_max) {
        x_max = object.centre_x + object.bbox_width / 2;
      }
      if (object.centre_y - object.bbox_height / 2 < y_min) {
        y_min = object.centre_y - object.bbox_height / 2;
      }
      if (object.centre_y + object.bbox_height / 2 > y_max) {
        y_max = object.centre_y + object.bbox_height / 2;
      } 
  }

  ROS_INFO("x_max is %f, x_min is %f, y_max is %f, y_min is %f", x_max, x_min, y_max, y_min);
  if (x_min == 10000 || y_min == 10000 || y_max == -1 || x_max == -1) {
    ROS_INFO("no pic seen");
    return BT::NodeStatus::FAILURE;
  } else {
    area = (y_max - y_min) * (x_max - x_min);
    ROS_INFO("area is %f", area);
    if (area >= area_benchmark) {
      ROS_INFO("see full bin");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("no full bin");
      return BT::NodeStatus::FAILURE;
    }  
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsFullBinSeenCondition>("IsFullBinSeen");
}