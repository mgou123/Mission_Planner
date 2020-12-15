#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include <asv_utils/geometry_utils.h>

#include "behavior_tree/plugins/action/gate_pair_action.hpp"

namespace behavior_tree
{
GatePairAction::GatePairAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{}

BT::NodeStatus GatePairAction::tick() 
{
    geometry_msgs::PoseStamped gate_1, gate_2;
    geometry_msgs::Pose robot_pose, gate_1_pose, gate_2_pose;
    double desired_heading;

    if (!getInput("gate_1", gate_1)) {
        ROS_ERROR("[PairGate] Gate 1 not available");
        return BT::NodeStatus::FAILURE;
    }  

    if (!getInput("gate_1", gate_1)) {
        ROS_ERROR("[PairGate] Gate 2 not available");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_ERROR("[PairGate] Pose not available");
        return BT::NodeStatus::FAILURE;
    }

    gate_1_pose = gate_1.pose;
    gate_2_pose = gate_2.pose;

    double x_mid = (gate_1_pose.position.x + gate_2_pose.position.x) / 2; 
    double y_mid = (gate_1_pose.position.y + gate_2_pose.position.y) / 2;

    desired_heading = (M_PI / 2) - atan2(x_mid, y_mid) + geometry_utils::yawFromQuaternion(robot_pose.orientation);

    setOutput("desired_heading", geometry_utils::yawToQuaternion(desired_heading));

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::GatePairAction>("GatePair");
}
