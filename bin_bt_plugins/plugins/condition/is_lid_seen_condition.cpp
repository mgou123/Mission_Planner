#include <string>
#include <memory>

#include "bin_bt_plugins/condition/is_lid_seen_condition.hpp"

namespace mp_behavior_tree
{

IsLidSeenCondition::IsLidSeenCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsLidSeenCondition::tick()
{
  int lid_num = 0;
  int count = 0;
  std::vector<bb_msgs::DetectedObject> objects;
  float area_1 = -1;
  float area_2 = -1;
  float area_benchmark;

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
  
  if (!getInput("area_benchmark", area_benchmark)) {
      ROS_ERROR("[IsLidSeen] area_benchmark not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("area_benchmark", area_benchmark);
  }

  for (auto object : objects) {
      if (object.name == "Lid") {
          ROS_INFO("one more lid");
          if (count == 0) {
              area_1 = object.bbox_area;
          } else if (count == 1) {
              area_2 = object.bbox_area;
          }
          count = count + 1;
      }
  }

  ROS_INFO("is_lid_seen running");
  ROS_INFO("count is %d, area_1 is %f, area_2 is %f", count, area_1, area_2);

  if (lid_num == 1 && area_1 > area_benchmark) {
      ROS_INFO("see enough lids");
      return BT::NodeStatus::SUCCESS;
  } else if (lid_num == 2 && area_1 > area_benchmark && area_2 > area_benchmark) {
      ROS_INFO("see enough lids");
      return BT::NodeStatus::SUCCESS;
  } else {
      ROS_INFO("not enough lids");
      return BT::NodeStatus::FAILURE;
  }
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsLidSeenCondition>("IsLidSeen");
}