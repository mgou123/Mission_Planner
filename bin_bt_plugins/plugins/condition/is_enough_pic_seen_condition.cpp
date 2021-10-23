#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_enough_pic_seen_condition.hpp"

namespace mp_behavior_tree
{

IsEnoughPicSeenCondition::IsEnoughPicSeenCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsEnoughPicSeenCondition::tick()
{   
  int pic_num = 0; 
  int count = 0; 
  bb_msgs::DetectedObjects objects; 

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[EnoughPicSeen] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("vision_objects", objects);
  }
  
  if (!getInput("pic_num", pic_num)) {
      ROS_ERROR("[EnoughPicSeen] pic_num not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("pic_num", pic_num);
  }

  for (auto object : objects.detected) {
      count += 1; 
  }

  ROS_INFO("enough_pic_seen running");
//   ROS_INFO("count is %d", count);
//   ROS_INFO("pic_num is %d", );

  if (count >= pic_num) {
      ROS_INFO("see enough pics");
      return BT::NodeStatus::SUCCESS;
  } else {
      ROS_INFO("not enough pics");
      return BT::NodeStatus::FAILURE;
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsEnoughPicSeenCondition>("IsEnoughPicSeen");
}