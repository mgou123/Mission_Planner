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
    geometry_msgs::PoseStamped goal_pose_stamped;
    tf2::Quaternion quat;
    double roll, pitch, yaw;

    if (!getInput("goal", goal_pose_stamped)) {
        ROS_ERROR("[NavigateToPose] goal not provided!");
        return;
    }

    tf2::convert(goal_pose_stamped.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    goal_.forward_setpoint = goal_pose_stamped.pose.position.y;
    goal_.sidemove_setpoint = goal_pose_stamped.pose.position.x;
    goal_.depth_setpoint = goal_pose_stamped.pose.position.z;

    goal_.yaw_setpoint = yaw;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mp_behavior_tree::NavigateToPose>(
        name, "locomotion", config);
    };

  factory.registerBuilder<mp_behavior_tree::NavigateToPose>(
    "Locomotion", builder);
}