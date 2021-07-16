#include <memory>
#include <string>

#include "vision_bt_plugins/action/set_object_goal.hpp"

namespace mp_behavior_tree 
{
SetObjectGoal::SetObjectGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus SetObjectGoal::tick() {
  vision::DetectedObjects objects;
  std::string target_identity;
  // -1: not found (empty)
  // 0: not found (default behavior)
  // 1: found (as expected).
  int targetFoundFlag = -1;

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[SetObjectGoal] objects not provided!");
      return BT::NodeStatus::FAILURE;
  }

  if (objects.detected.size() <= 0) {
    ROS_WARN("[SetObjectGoal] no objects received from vision!");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("target_identity", target_identity)) {
    ROS_WARN("[SetObjectGoal] target not provided! Defaulting to first object!");
  }


  ROS_INFO("[SetObjectGoal] Finding %s", target_identity.c_str());
  for (auto object : objects.detected) {
    ROS_INFO("[SetObjectGoal]: Object Name: %s", object.name.c_str());
    ROS_INFO("[SetObjectGoal]: Found? : %d", object.name.compare(target_identity) == 0);
    if (object.name.compare(target_identity) == 0)
    {
      targetFoundFlag = 1;  // As expected

      geometry_msgs::PoseStamped output_pose;
      output_pose.pose.position.x = (double)(object.rel_coords[0]);
      output_pose.pose.position.y = (double)(object.rel_coords[1]);
      output_pose.pose.position.z = (double)(object.rel_coords[2]);
      setOutput("goal", output_pose);
      setOutput("absolute_depth", (double)(object.world_coords[2]));
      setOutput("relative_yaw", (double)(object.angle));

      return BT::NodeStatus::SUCCESS;
    }
  }

  ROS_WARN("[SetObjectGoal]: No correct object found!");

  return BT::NodeStatus::FAILURE;  // Should never reach here, but just in case
}

}  // namespace mp_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<mp_behavior_tree::SetObjectGoal>("SetObjectGoal");
}
