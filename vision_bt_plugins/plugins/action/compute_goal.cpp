#include <memory>
#include <string>

#include "vision_bt_plugins/action/compute_goal.hpp"

namespace mp_behavior_tree
{
  ComputeGoal::ComputeGoal(
      const std::string &xml_tag_name,
      const BT::NodeConfiguration &conf)
      : BT::SyncActionNode(xml_tag_name, conf) {}

  BT::NodeStatus ComputeGoal::tick()
  {
    std::vector<bb_msgs::DetectedObject> objects;
    std::string target_identity;
    std::string main_target;
    geometry_msgs::PoseStamped goalRange;
    double range;

    // -1: not found (empty)
    // 0: not found (default behavior)
    // 1: found (as expected).
    int targetFoundFlag = -1;

    if (!getInput("vision_objects", objects))
    {
      ROS_ERROR("[ComputeGoal] objects not provided!");
      return BT::NodeStatus::FAILURE;
    }

    if (objects.size() <= 0)
    {
      ROS_WARN("[ComputeGoal] no objects received from vision!");
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("target_identity", target_identity))
    {
      ROS_WARN("[ComputeGoal] target not provided! Defaulting to first object!");
    }

    if (!getInput("main_target", main_target))
    {
      ROS_WARN("[ComputeGoal] Main target not provided! Defaulting to given range");
    }

    if (!main_target.empty())
    {
      // Finding main_target
      ROS_INFO("[ComputeGoal] Finding %s", main_target.c_str());
      for (auto object : objects)
      {
        std::string name = object.name;
        bool isMainTarget = object.name.compare(main_target) == 0;

        ROS_INFO("[ComputeGoal]: Object Name: %s", name.c_str());
        ROS_INFO("[ComputeGoal]: Found? : %d", isMainTarget);
        if (isMainTarget)
        {
          targetFoundFlag = 1;
          range = (double)object.rel_coords[0];
        }
      }

      if (targetFoundFlag == -1)
      {
        ROS_ERROR("[ComputeGoal]: Main Target not found!");
        return BT::NodeStatus::FAILURE;
      }

      targetFoundFlag = -1;
    }
    else if (getInput("range", goalRange))
    {
      targetFoundFlag = 1;
      range = goalRange.pose.position.x;
    }
    else
    {
      ROS_ERROR("[ComputeGoal] Range is also not provided!");
      return BT::NodeStatus::FAILURE;
    }

    // Finding target_identity
    ROS_INFO("[ComputeGoal] Finding %s", target_identity.c_str());
    for (auto object : objects)
    {
      ROS_INFO("[ComputeGoal]: Object Name: %s", object.name.c_str());
      ROS_INFO("[ComputeGoal]: Found? : %d", object.name.compare(target_identity) == 0);
      // TODO: Check that hole belong's to main_target
      if (object.name.compare(target_identity) == 0)
      {
        targetFoundFlag = 1; // As expected

        geometry_msgs::PoseStamped output_pose;
        ROS_INFO("[ComputeGoal]: Passed move_coords is : %d", object.move_coords);

        if (object.move_coords == 2)
        { // dx dy dz
          ROS_WARN("[ComputeGoal]: Incorrect move_coords for this node. Behaving like SetObjectGoal");
          output_pose.pose.position.x = (double)(object.rel_coords[0]);
          output_pose.pose.position.y = (double)(object.rel_coords[1]);
          output_pose.pose.position.z = (double)(-object.world_coords[2]); // Flip this sign to negative

          setOutput("goal", output_pose);
          setOutput("absolute_depth", (double)(-object.world_coords[2]));
          setOutput("absolute_yaw", (double)(object.world_yaw));

          ROS_INFO("[ComputeGoal]: Forward move is : %f", (double)(object.rel_coords[0]));
          ROS_INFO("[ComputeGoal]: Sidemove is : %f", (double)(object.rel_coords[1]));
          ROS_INFO("[ComputeGoal]: Depth move is : %f", (double)(object.rel_coords[2]));

          ROS_INFO("[ComputeGoal]: Depth is : %f", (double)(-object.world_coords[2]));
          ROS_INFO("[ComputeGoal]: Yaw is : %f", (double)(object.angle));
          ROS_INFO("[ComputeGoal]: World Yaw is : %f", (double)(object.world_yaw));
        }
        else if (object.move_coords == 4)
        { // theta phi
          double theta = (double)object.rel_coords[0];
          double phi = (double)object.rel_coords[1];

          double sidemove = range * std::tan((theta + 1e-5) * 3.14159265 / 180.0);
          double height = -1 * range * std::tan((phi + 1e-5) * 3.14159265 / 180.0);

          output_pose.pose.position.x = 0;
          output_pose.pose.position.y = sidemove;
          output_pose.pose.position.z = height;

          setOutput("goal", output_pose);

          ROS_INFO("[ComputeGoal]: Range to main target is : %f", range);
          ROS_INFO("[ComputeGoal]: Centering Yaw is : %f", theta);
          ROS_INFO("[ComputeGoal]: Centering Pitch is : %f", phi);
          ROS_INFO("[ComputeGoal]: Required sidemove is : %f", sidemove);
          ROS_INFO("[ComputeGoal]: Required rel. depth is : %f", height);
        }
        else
        {
          ROS_ERROR("[ComputeGoal]: Unknown move_coords. Implement it first!");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
      }
    }

    ROS_WARN("[ComputeGoal]: No correct object found!");
    return BT::NodeStatus::FAILURE;
  }

} // namespace mp_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::ComputeGoal>("ComputeGoal");
}
