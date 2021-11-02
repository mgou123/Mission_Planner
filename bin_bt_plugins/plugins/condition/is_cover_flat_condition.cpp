#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_cover_flat_condition.hpp"

namespace mp_behavior_tree
{
IsCoverFlatCondition::IsCoverFlatCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsCoverFlatCondition::tick()
{   
  std::vector<bb_msgs::DetectedObject> objects;
  bb_msgs::DetectedObject cover_1;
  bb_msgs::DetectedObject cover_2;
  std::string detector_name;
  float angle;
  float error_range;
  int count = 0;
  float lower_bound;
  float upper_bound;
  float overflow_upper = -90.5;
  float overflow_lower = 90.5;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsCoverFlat] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("angle", angle)) {
    ROS_ERROR("[IsCoverFlat] angle not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("angle", objects);
  }

  if (!getInput("error_range", error_range)) {
    ROS_ERROR("[IsCoverFlat] error_range not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("error_range", error_range);
  }

  if (!getInput("detector_name", detector_name)) {
    ROS_ERROR("[IsCoverFlat] detector_name not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("detector_name", detector_name);
  }

  lower_bound = angle - error_range;
  upper_bound = angle + error_range;

  if (lower_bound < -90) {
    overflow_lower = lower_bound + 180;
  }

  if (upper_bound > 90) {
    overflow_upper = upper_bound - 180;
  }

  ROS_INFO("Is cover flat running");

  for (auto object : objects) {
    //ROS_INFO("1");
    if (object.name.compare(detector_name) == 0) {
      if (count == 0) {
        count ++;
        cover_1 = object;
        ROS_INFO("cover1 angle is %d", cover_1.angle); 
      } else if (count == 1) {
        count ++;
        cover_2 = object;
        ROS_INFO("cover2 angle is %d", cover_2.angle); 
      }
    }
  }
  
  if (count < 1) {
    ROS_INFO("not enough covers seen");
    return BT::NodeStatus::FAILURE;
  }

  if (count == 2) {
    if ((cover_1.angle > lower_bound || cover_1.angle > overflow_lower) && (cover_1.angle < upper_bound || cover_1.angle < overflow_upper) &&
        (cover_2.angle > lower_bound || cover_2.angle > overflow_lower) && (cover_2.angle < upper_bound || cover_2.angle < overflow_upper)) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  } else if (count == 1) {
    if ((cover_1.angle > lower_bound || cover_1.angle > overflow_lower) && (cover_1.angle < upper_bound || cover_1.angle < overflow_upper)) {
       return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  } 
  return BT::NodeStatus::FAILURE;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsCoverFlatCondition>("IsCoverFlat");
}