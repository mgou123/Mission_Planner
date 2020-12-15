#include "ros/ros.h"

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/condition/is_goal_reached_condition.hpp"

namespace behavior_tree
{

IsGoalReachedCondition::IsGoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    max_error_(0.1),
    dur_tol_(1.0)
{
    getInput("max_error", max_error_);
    getInput("duration_tol", dur_tol_);
}

BT::NodeStatus IsGoalReachedCondition::tick()
{
    ros::Time now = ros::Time::now();
    geometry_msgs::Pose goal;
    geometry_msgs::Pose robot_pose;
    ros::Time last_pose_update;


    if (!getInput("goal", goal)) {
        ROS_WARN("[IsGoalReached] No goal available");
        return BT::NodeStatus::SUCCESS;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_WARN("[IsGoalReached] No pose available");
        return BT::NodeStatus::SUCCESS;
    }

    if (!getInput("last_pose_update", last_pose_update)) {
        ROS_WARN("[IsGoalReached] Cannot get timestamp of last pose update");
        return BT::NodeStatus::SUCCESS;
    }

    if (now - last_pose_update > ros::Duration(dur_tol_)) {
        ROS_WARN("[IsGoalReached] Too long since last odom update");
        return BT::NodeStatus::SUCCESS;
    }

    if (geometry_utils::euclidean_distance(robot_pose, goal) <= max_error_) {
        return BT::NodeStatus::SUCCESS;
    }

    //plan a route
    return BT::NodeStatus::FAILURE;
}


} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::IsGoalReachedCondition>("IsGoalReached");
}
