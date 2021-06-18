#include <memory>
#include <string>

#include "auv_bt_plugins/plugins/action/locomotion_action.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace mp_behavior_tree
{

LocomotionAction::LocomotionAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav_utils::action::Locomotion>(xml_tag_name, action_name, conf) {}

void LocomotionAction::on_tick() {
    geometry_msgs::msg::PoseStamped goal_pose_stamped;
    tf2::Quaternion quat;
    double roll, pitch, yaw;

    if (!getInput("goal", goal_pose)) {
        ROS_ERROR("[LocomotionAction] goal not provided!");
        return;
    }

    tf2::fromMsg(goal_pose_stamped.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    goal_.forward_setpoint = goal_pose_stamped.pose.position.y;
    goal_.sidemove_setpoint = goal_pose_stamped.pose.position.x;
    goal_.depth_setpoint = goal_pose_stamped.pose.position.z;

    goal_.heading_setpoint = yaw;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mp_behavior_tree::LocomotionAction>(
        name, "locomotion", config);
    };

  factory.registerBuilder<mp_behavior_tree::LocomotionAction>(
    "Locomotion", builder);
}