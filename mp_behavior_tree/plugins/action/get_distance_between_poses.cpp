#include "mp_behavior_tree/plugins/action/get_distance_between_poses.hpp"

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mp_behavior_tree
{
GetDistanceBetweenPoses::GetDistanceBetweenPoses(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf) 
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus GetDistanceBetweenPoses::tick() {
    geometry_msgs::PoseStamped source_pose;
    geometry_msgs::PoseStamped target_pose;

    double distance;

    if (!getInput("source_pose", source_pose)) {
        ROS_WARN("[GetYawBetweenPoses] Cannot get source pose!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("target_pose", target_pose)) {
        ROS_WARN("[GetYawBetweenPoses] Cannot get target pose!");
        return BT::NodeStatus::FAILURE;
    }

    double tgt_x = target_pose.pose.position.x;
    double src_x = source_pose.pose.position.x;
    double tgt_y = target_pose.pose.position.y;
    double src_y = source_pose.pose.position.y;

    distance = sqrt((tgt_x - src_x) * (tgt_x - src_x) + (tgt_y - src_y) * (tgt_y - src_y));

    geometry_msgs::PoseStamped relative_pose;
    relative_pose.pose.position.x = distance;

    setOutput("relative_pose", relative_pose);
    setOutput("distance", distance);

    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::GetDistanceBetweenPoses>("GetDistanceBetweenPoses");
}