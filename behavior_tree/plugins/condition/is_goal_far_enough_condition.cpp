#include "ros/ros.h"

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/condition/is_goal_far_enough_condition.hpp"

namespace behavior_tree
{

IsGoalFarEnoughCondition::IsGoalFarEnoughCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    clearance_(20.0)
{
    getInput("clearance", clearance_);
}

BT::NodeStatus IsGoalFarEnoughCondition::tick()
{
    geometry_msgs::Pose goal;
    geometry_msgs::Pose robot_pose;
    ros::Time last_pose_update;


    if (!getInput("goal", goal)) {
        ROS_ERROR("[IsGoalFarEnough] No goal available");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_ERROR("[IsGoalFarEnough] No pose available");
        return BT::NodeStatus::FAILURE;
    }

    if (geometry_utils::euclidean_distance(robot_pose, goal) >= clearance_) {
        return BT::NodeStatus::SUCCESS;
    }

    //go for a mid goal
    return BT::NodeStatus::FAILURE;
}


} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::IsGoalFarEnoughCondition>("IsGoalFarEnough");
}
