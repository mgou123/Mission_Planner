#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include <asv_utils/geometry_utils.h>

#include "behavior_tree/plugins/action/non_red_gate_action.hpp"

namespace behavior_tree
{
NonRedGateAction::NonRedGateAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{
    getInput("offset_distance", offset_distance_);
}

BT::NodeStatus NonRedGateAction::tick() 
{
    geometry_msgs::PoseStamped gate_1;
    geometry_msgs::Pose robot_pose, gate_1_pose;

    if (!getInput("gate_1", gate_1)) {
        ROS_ERROR("[NonRedGate] No gate available");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_ERROR("[NonRedGate] Pose not available");
        return BT::NodeStatus::FAILURE;
    }

    gate_1_pose = gate_1.pose;

    double desired_heading = (M_PI / 2) - atan2(gate_1_pose.position.x, gate_1.pose.position.y + offset_distance_) + geometry_utils::yawFromQuaternion(robot_pose.orientation);
    setOutput("desired_heading", geometry_utils::yawToQuaternion(desired_heading));

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::NonRedGateAction>("NonRedGate");
}
