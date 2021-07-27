#include "auv_bt_plugins/action/modify_goal.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mp_behavior_tree {
  ModifyGoal::ModifyGoal(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf
  ) : BT::SyncActionNode(xml_tag_name, conf){}

  BT::NodeStatus ModifyGoal::tick() {
    geometry_msgs::PoseStamped original_pose;
    geometry_msgs::PoseStamped modifying_pose;
    geometry_msgs::PoseStamped final_pose;
    std::vector<std::string> zeroed_comps;

    if(getInput("original_pose", original_pose)) {
      if(getInput("modifying_pose", modifying_pose) && getInput("zeroed_components", zeroed_comps)) {
        ROS_WARN("[ModifyGoal] Ambiguous: both modifying_pose and zeroed_components provided!");
        return BT::NodeStatus::FAILURE;
      } else if (!getInput("modifying_pose", modifying_pose) && !getInput("zeroed_components", zeroed_comps)) {
          ROS_WARN("[ModifyGoal] Both modifying_pose and zeroed_components are empty!");
          return BT::NodeStatus::FAILURE;
      } else if (getInput("modifying_pose", modifying_pose)) {
          ROS_INFO("[ModifyGoal] Adding modifying_pose to original pose...");
          final_pose.pose.position.x = original_pose.pose.position.x + modifying_pose.pose.position.x;
          final_pose.pose.position.y = original_pose.pose.position.y + modifying_pose.pose.position.y;
          final_pose.pose.position.z = original_pose.pose.position.z + modifying_pose.pose.position.z; 

          std::cout << "Original Pose is " << original_pose.pose.position.x << ", " << original_pose.pose.position.y << ", " << original_pose.pose.position.z;
          std::cout << "Modifying Pose is " << modifying_pose.pose.position.x << ", " << modifying_pose.pose.position.y << ", " << modifying_pose.pose.position.z;
          std::cout << "Final Pose is " << final_pose.pose.position.x << ", " << final_pose.pose.position.y << ", " << final_pose.pose.position.z;

          setOutput("final_pose", final_pose);
          return BT::NodeStatus::SUCCESS;
      } else {
          ROS_INFO("[ModifyGoal] Zeroing out components...");
          final_pose.pose.position.x = original_pose.pose.position.x;
          final_pose.pose.position.y = original_pose.pose.position.y;
          final_pose.pose.position.z = original_pose.pose.position.z;        

          for (std::string comps : zeroed_comps) {
            if (comps == "x") {
              final_pose.pose.position.x = 0;
            } else if (comps == "y") {
              final_pose.pose.position.y = 0;
            } else if (comps == "z") {
              final_pose.pose.position.z = 0;
            } else {
              ROS_WARN("[ModifyGoal] Unrecognized components %s! Skipping...", comps.c_str());
            }
          }

          setOutput("final_pose", final_pose);
          return BT::NodeStatus::SUCCESS;
      }
    } else {
      ROS_WARN("[ModifyGoal] Original Pose not given!");
      return BT::NodeStatus::FAILURE;
    }
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::ModifyGoal>("ModifyGoal");
}