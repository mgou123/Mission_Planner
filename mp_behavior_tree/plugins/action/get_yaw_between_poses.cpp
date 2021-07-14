#include "mp_behavior_tree/plugins/action/get_yaw_between_poses.hpp"

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mp_behavior_tree
{
GetYawBetweenPoses::GetYawBetweenPoses(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf) 
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus GetYawBetweenPoses::tick() {
    geometry_msgs::PoseStamped source_pose;
    geometry_msgs::PoseStamped target_pose;

    double absolute_yaw, relative_yaw;


    if (!getInput("source_pose", source_pose)) {
        ROS_WARN("[GetYawBetweenPoses] Cannot get source pose!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("target_pose", target_pose)) {
        ROS_WARN("[GetYawBetweenPoses] Cannot get target pose!");
        return BT::NodeStatus::FAILURE;
    }

    absolute_yaw = atan2(target_pose.pose.position.y - source_pose.pose.position.y,
                                target_pose.pose.position.x - source_pose.pose.position.x);
    absolute_yaw = absolute_yaw * 180.0 / M_PI;

    setOutput("absolute_yaw", absolute_yaw);

    tf2::Quaternion source_quat;
    double roll, pitch, yaw;
    tf2::convert(source_pose.pose.orientation, source_quat);
    tf2::Matrix3x3(source_quat).getRPY(roll, pitch, yaw);
    relative_yaw = absolute_yaw - yaw;

    if (relative_yaw < 0) {
        relative_yaw += 360.0;
    }

    setOutput("relative_yaw", relative_yaw);

    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::GetYawBetweenPoses>("GetYawBetweenPoses");
}