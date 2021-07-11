#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mp_behavior_tree/plugins/action/set_relative_goal.hpp"

namespace mp_behavior_tree
{

SetRelativeGoal::SetRelativeGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus SetRelativeGoal::tick() {
    geometry_msgs::PoseStamped relative_goal, current_pose, goal;
    double yaw, depth;
    if (!getInput("relative_goal", relative_goal)) {
        ROS_ERROR("[SetRelativeGoal] Unable to get relative goal!");
        return BT::NodeStatus::FAILURE;
    } else if (!getInput("pose", current_pose)) {
        ROS_ERROR("[SetRelativeGoal] Unable to get current pose!");
        return BT::NodeStatus::FAILURE;
    }

    goal.pose.position.x = current_pose.pose.position.x + relative_goal.pose.position.x;
    goal.pose.position.y = current_pose.pose.position.y + relative_goal.pose.position.y;
    goal.pose.position.z = current_pose.pose.position.z + relative_goal.pose.position.z;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(current_pose.pose.orientation, q_orig);
    tf2::convert(relative_goal.pose.orientation, q_rot);
    q_new = q_orig * q_rot;
    q_new.normalize();
    tf2::convert(q_new, goal.pose.orientation);

    if (getInput("depth_lock", depth)) {
        goal.pose.position.z = depth;
    }

    if (getInput("yaw_lock", yaw)) {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw / 180 * M_PI);
        quat = quat.normalize();
        tf2::convert(quat, goal.pose.orientation);
    }

    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SetRelativeGoal>("SetRelativeGoal");
}