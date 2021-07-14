#include <string>
#include <memory>

#include "mp_behavior_tree/plugins/condition/is_lid_seen.hpp"

namespace mp_behavior_tree
{

IsLidSeenCondition::IsLidSeenCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsLidSeenCondition::tick()
{   
  int lid_num; 
  int count;
  vision::DetectedObjects objects; 

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[IsLidSeen] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("vision_objects", objects);
  }
  
  if (!getInput("lid_num", lid_num)) {
      ROS_ERROR("[IsLidSeen] lid_num not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("lid_num", lid_num);
  }

  for (auto object : objects.detected) {
      if (object.name == "Lid") {
          count = count + 1;
      }
  }
    
  if (count >= lid_num) {
      return BT::NodeStatus::SUCCESS;
  } else {
      return BT::NodeStatus::FAILURE;
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsLidSeenCondition>("IsLidSeen");
}