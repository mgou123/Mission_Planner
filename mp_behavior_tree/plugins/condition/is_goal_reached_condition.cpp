#include <string>
#include <memory>

#include "mp_behavior_tree/plugins/condition/is_goal_reached_condition.hpp"

namespace mp_behavior_tree
{

IsGoalReachedCondition::IsGoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    goal_reached_tol_(0.1),
    odom_update_timeout_(1.0) {

    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    node_->param("goal_reached_tol", goal_reached_tol_, 0.1);
    node_->param("odom_update_timeout", odom_update_timeout_, 1.0);
}

BT::NodeStatus IsGoalReachedCondition::tick()
{
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot_pose;
    ros::Time last_pose_update;

    if (!getInput("goal", goal)) {
        ROS_WARN("[IsGoalReached] No goal available");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_WARN("[IsGoalReached] No pose available");
        return BT::NodeStatus::FAILURE;
    }

    // if (now - last_pose_update > ros::Duration(odom_update_timeout_)) {
    //     ROS_WARN("[IsGoalReached] Too long since last odom update");
    //     return BT::NodeStatus::FAILURE;
    // } // change to using the timestamp

    double dx = goal.pose.position.x - robot_pose.pose.position.x;
    double dy = goal.pose.position.y - robot_pose.pose.position.y;
    double dz = goal.pose.position.z - robot_pose.pose.position.z;

    if ((dx * dx + dy * dy + dz * dz) > (goal_reached_tol_ * goal_reached_tol_)) {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsGoalReachedCondition>("IsGoalReached");
}