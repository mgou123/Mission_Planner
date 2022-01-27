#include "bb_bt_plugins/action/go_to.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace mp_behavior_tree
{
GoTo::GoTo(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<bb_msgs::LocomotionAction, 
                 bb_msgs::LocomotionGoal, 
                 bb_msgs::LocomotionResult>(xml_tag_name, action_name, conf) {}

void GoTo::on_tick() {
    tf2::Quaternion quat;
    geometry_msgs::PoseStamped goal_pose_stamped;
    double depth;
    double roll, pitch, yaw;
    bool rel;

    double temp;

    if (!getInput("goal", goal_pose_stamped)) {
      double forward, sideways, depth, yaw;
      
      if (!getInput("forward", forward)) forward = 0;
      if (!getInput("sideways", sideways)) sideways = 0;
      if (!getInput("depth", depth)) depth = 0;
      if (!getInput("yaw", yaw)) yaw = 0;

      goal_.forward_setpoint = forward;
      goal_.sidemove_setpoint = sideways;
      goal_.depth_setpoint = depth;
      goal_.yaw_setpoint = yaw;

      bool rel;
      if (getInput("relative", rel) && !rel) ROS_WARN("[GoTo] Absolute motion can only be specified with goal pose! Default to relative motion.");

      goal_.yaw_rel = true;
      goal_.movement_rel = true;

    } else {
      if (getInput("forward", temp)) {
        ROS_WARN("[GoTo] input values specified for both goal and forward! Ignoring forward value...");
      } else if (getInput("sideways", temp)) {
        ROS_WARN("[GoTo] input values specified for both goal and sideways! Ignoring sideways value...");
      } else if (getInput("depth", temp)) {
        ROS_WARN("[GoTo] input values specified for both goal and depth! Ignoring depth value...");
      } else if (getInput("yaw", temp)) {
        ROS_WARN("[GoTo] input values specified for both goal and yaw! Ignoring yaw value...");
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
      getInput("relative", relative);

      if (relative) {
          goal_.movement_rel = true;
          goal_.yaw_rel = true;
      } else {
          goal_.movement_rel = false;
          goal_.yaw_rel = false;
      }
    }

    if (getInput("depth_lock", depth)) {
      if (getInput("depth", temp)) ROS_WARN("[GoTo] Both depth and depth_lock value set! Defaulting to depth_lock value...");
      goal_.depth_setpoint = depth;
      
    }

    if (getInput("yaw_lock", yaw)) {
      if (getInput("yaw", temp)) ROS_WARN("[GoTo] Both yaw and yaw_lock value set! Defaulting to yaw_lock value...");
      goal_.yaw_rel = false;
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
      return std::make_unique<mp_behavior_tree::GoTo>(
        name, "Locomotion", config);
    };

  factory.registerBuilder<mp_behavior_tree::GoTo>(
    "GoTo", builder);
}