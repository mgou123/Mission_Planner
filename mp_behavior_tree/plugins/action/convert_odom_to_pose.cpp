#include "mp_behavior_tree/plugins/action/convert_odom_to_pose.hpp"

namespace mp_behavior_tree
{
ConvertOdomToPose::ConvertOdomToPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf) 
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus ConvertOdomToPose::tick() {
    nav_msgs::Odometry odom_pose;
    geometry_msgs::PoseStamped pose;
    if (!getInput("odom_pose", odom_pose)) {
        ROS_WARN("[ConvertOdomToPose] Cannot get odom_pose!");
        return BT::NodeStatus::FAILURE;
    }

    pose.pose = odom_pose.pose.pose;

    setOutput("pose", pose);

    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::ConvertOdomToPose>("ConvertOdomToPose");
}