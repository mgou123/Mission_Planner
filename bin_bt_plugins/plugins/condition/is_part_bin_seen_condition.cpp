#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_part_bin_seen_condition.hpp"

namespace mp_behavior_tree
{

IsPartBinSeenCondition::IsPartBinSeenCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsPartBinSeenCondition::tick()
{   
  std::vector<bb_msgs::DetectedObject> objects; 
  int count = 0; 

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[IsPartBinSeen] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("vision_objects", objects);
  }

  for (auto object : objects) {
    count++; 
  }

  if (count > 0) {
      return BT::NodeStatus::SUCCESS;
  } else {
      return BT::NodeStatus::FAILURE;
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsPartBinSeenCondition>("IsPartBinSeen");
}