#include <memory>
#include <string>

#include "vision_bt_plugins/action/set_object_goal.hpp"

namespace mp_behavior_tree
{
  SetObjectGoal::SetObjectGoal(
    const std::string& xml_tag_name,
    const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {}

  BT::NodeStatus SetObjectGoal::tick()
  {
    bb_msgs::DetectedObjects objects;
    std::string target_identity;
    // -1: not found (empty)
    // 0: not found (default behavior)
    // 1: found (as expected).
    int targetFoundFlag = -1;

    if (!getInput("vision_objects", objects))
    {
      ROS_ERROR("[SetObjectGoal] objects not provided!");
      return BT::NodeStatus::FAILURE;
    }

    if (objects.detected.size() <= 0)
    {
      ROS_WARN("[SetObjectGoal] no objects received from vision!");
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("target_identity", target_identity))
    {
      ROS_WARN("[SetObjectGoal] target not provided! Defaulting to first object!");
    }

    ROS_INFO("[SetObjectGoal] Finding %s", target_identity.c_str());
    for (auto object : objects.detected)
    {
      ROS_INFO("[SetObjectGoal]: Object Name: %s", object.name.c_str());
      ROS_INFO("[SetObjectGoal]: Found? : %d", object.name.compare(target_identity) == 0);
      if (object.name.compare(target_identity) == 0)
      {
        targetFoundFlag = 1; // As expected

        geometry_msgs::PoseStamped output_pose;
        ROS_INFO("[SetObjectGoal]: Passed move_coords is : %d", object.move_coords);

        if (object.move_coords == 2)
        { // dx dy dz
          output_pose.pose.position.x = (double)(object.rel_coords[0]);
          output_pose.pose.position.y = (double)(object.rel_coords[1]);    // Use 2 for now..
          output_pose.pose.position.z = (double)(-object.world_coords[2]); // Flip this sign to negative

          setOutput("goal", output_pose);
          setOutput("absolute_depth", (double)(-object.world_coords[2]));
          setOutput("absolute_yaw", (double)(object.world_yaw));

          ROS_INFO("[SetObjectGoal]: Forward move is : %f", (double)(object.rel_coords[0]));
          ROS_INFO("[SetObjectGoal]: Sidemove is : %f", (double)(object.rel_coords[1]));
          ROS_INFO("[SetObjectGoal]: Depth move is : %f", (double)(object.rel_coords[2]));

          ROS_INFO("[SetObjectGoal]: Depth is : %f", (double)(-object.world_coords[2]));
          ROS_INFO("[SetObjectGoal]: Yaw is : %f", (double)(object.angle));
          ROS_INFO("[SetObjectGoal]: World Yaw is : %f", (double)(object.world_yaw));
        }
        else if (object.move_coords == 4)
        { // theta phi
          ROS_INFO("[SetObjectGoal]: Centering Yaw is : %f", (double)(object.rel_coords[0]));
          setOutput("absolute_yaw", (double)(object.rel_coords[0]));
        }
        else
        {
          ROS_ERROR("[SetObjectGoal]: Unknown move_coords. Implement it first!");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
      }
    }

    ROS_WARN("[SetObjectGoal]: No correct object found!");

    return BT::NodeStatus::FAILURE; // Should never reach here, but just in case
  }

} // namespace mp_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SetObjectGoal>("SetObjectGoal");
}
