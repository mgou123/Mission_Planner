#include <memory>
#include <string>

#include "auv_bt_plugins/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace mp_behavior_tree
{

NavigateToPose::NavigateToPose(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav_utils::LocomotionAction, 
                 nav_utils::LocomotionGoal, 
                 nav_utils::LocomotionResult>(xml_tag_name, action_name, conf) {}

void NavigateToPose::on_tick() {
    tf2::Quaternion quat;
    geometry_msgs::PoseStamped goal_pose_stamped;
    double depth;
    double roll, pitch, yaw;
    bool rel;

    if (!getInput("goal", goal_pose_stamped)) {
        ROS_ERROR("[NavigateToPose] goal not provided!");
        return;
    }

    // set goal
    tf2::convert(goal_pose_stamped.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    goal_.forward_setpoint = goal_pose_stamped.pose.position.x;
    goal_.sidemove_setpoint = goal_pose_stamped.pose.position.y;
    goal_.depth_setpoint = goal_pose_stamped.pose.position.z;

    goal_.yaw_setpoint = yaw / M_PI * 180.0;

    //set relative
    bool relative;
    if (getInput("relative", relative)) {
      if (relative) {
        goal_.movement_rel = true;
        goal_.yaw_rel = true;
      } else {
        goal_.movement_rel = false;
        goal_.yaw_rel = false;
      }
    } else {
      goal_.movement_rel = true;
      goal_.yaw_rel = true;
    } 

    if (!getInput("yaw_lock_relative", rel)) {
      ROS_WARN("[NavigateToPose]: yaw_lock_relative not passed. Defaults to false");
      rel = false;
    }

    if (getInput("depth_lock", depth)) {
        goal_.depth_setpoint = depth;
    }

    if (getInput("yaw_lock", yaw)) {
      goal_.yaw_rel = rel;
      goal_.yaw_setpoint = yaw;
    }

    double sidemove_tolerance, forward_tolerance, yaw_tolerance;

    if (getInput("sidemove_tolerance", sidemove_tolerance)) {
      goal_.sidemove_tolerance = sidemove_tolerance;
    }

    if (getInput("forward_tolerance", forward_tolerance)) {
      goal_.forward_tolerance = forward_tolerance;
    }

    if (getInput("yaw_tolerance", yaw_tolerance)) {
      goal_.yaw_tolerance = yaw_tolerance;
    }
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mp_behavior_tree::NavigateToPose>(
        name, "Locomotion", config);
    };

  factory.registerBuilder<mp_behavior_tree::NavigateToPose>(
    "NavigateToPose", builder);
}