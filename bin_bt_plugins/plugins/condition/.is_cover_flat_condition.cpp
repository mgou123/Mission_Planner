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
  bb_msgs::DetectedObjects objects;
  bb_msgs::DetectedObject cover_1;
  bb_msgs::DetectedObject cover_2;
  float angle_range; 
  int count = 0;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[IsCoverFlat] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("angle_range", angle_range)) {
    ROS_ERROR("[IsCoverFlat] angle_range not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("angle_range", objects);
  }

  ROS_INFO("Is cover flat running");

  for (auto object : objects.detected) {
    //ROS_INFO("1");
    if (object.name.compare("cover") == 0) {
      if (count == 0) {
        count ++;
        cover_1 = object; 
      } else if (count == 1) {
        count ++;
        cover_2 = object;
      }
    }
  }
  
  if (count < 2) {
    ROS_INFO("not enough covers seen");
    return BT::NodeStatus::FAILURE;
  }

  if (cover_1.bbox_height / cover_1.bbox_width < angle_range && cover_2.bbox_height / cover_2.bbox_width < angle_range) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsCoverFlatCondition>("IsCoverFlat");
}