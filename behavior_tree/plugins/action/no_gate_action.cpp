#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include "behavior_tree/plugins/action/no_gate_action.hpp"

namespace behavior_tree
{
NoGateAction::NoGateAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{
}

BT::NodeStatus NoGateAction::tick() 
{
    geometry_msgs::Pose robot_pose;

    if (!getInput("pose", robot_pose)) {
        ROS_ERROR("[NoGate] No pose available");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("desired_heading", robot_pose.orientation);

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::NoGateAction>("NoGate");
}
