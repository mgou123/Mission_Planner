#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "buoy_bt_plugins/action/find_buoy.hpp"

namespace mp_behavior_tree
{
FindBuoy::FindBuoy(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf) 
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus FindBuoy::tick() {
  vision::DetectedObjects objects;
  std::string target_identity;
  int targetFoundFlag = -1; // -1 means not found (empty). 0 means not found (default behavior). 1 means found (as expected).

  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[FindBuoy] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (objects.detected.size() <= 0) {
    ROS_WARN("[FindBuoy] no objects received from vision!");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("target_identity", target_identity)) {
    ROS_WARN("[FindBuoy] target not provided! Defaulting to first object!");
  }

  ROS_INFO("[FindBuoy] Finding %s", target_identity.c_str());
  for (auto object : objects.detected) {
    if (object.name == target_identity) {
      targetFoundFlag = 1; // As expected
      setOutput("forward", object.rel_coords[0]);
      setOutput("sideways", object.rel_coords[1]);
      setOutput("depth", object.rel_coords[2]);
      setOutput("yaw", 0.0F); // TODO: Set this properly

      return BT::NodeStatus::SUCCESS;
    }
  }

  if (targetFoundFlag == -1) {
    vision::DetectedObject defaultTarget = objects.detected[0]; // Default to first element
    setOutput("forward", defaultTarget.rel_coords[0]);
    setOutput("sideways", defaultTarget.rel_coords[1]);
    setOutput("depth", defaultTarget.rel_coords[2]);
    setOutput("yaw", 0.0F); // TODO: Set this properly

    ROS_INFO("Defaulting to first element. Forward: %f. Sideways: %f. Depth: %f", defaultTarget.rel_coords[0], defaultTarget.rel_coords[1], defaultTarget.rel_coords[2]);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE; // Should never reach here, but just in case
}

} // namespace mp_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::FindBuoy>("FindBuoy");
}