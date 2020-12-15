#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/action/set_next_goal_action.hpp"

namespace behavior_tree
{
SetNextGoalAction::SetNextGoalAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{
}

BT::NodeStatus SetNextGoalAction::tick() 
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose new_goal;

    if (!getInput("waypoints", waypoints)) {
        ROS_ERROR("Unable to obtain waypoints from blackboard in set_next_goal_action");
        return BT::NodeStatus::FAILURE;
    }

    if (waypoints.empty()) {
        ROS_WARN("No next goal available from waypoints");
        return BT::NodeStatus::FAILURE;
    }

    new_goal = waypoints[0];

    setOutput("goal", new_goal);

    if (first_time_) {
        first_time_ = false;
        goal_ = new_goal;
        return BT::NodeStatus::SUCCESS;
    }

    if (!geometry_utils::is_equal(goal_, new_goal)) {
        config().blackboard->set("goal_updated", true);
        ROS_INFO("[SetNextGoal] Setting a new goal...");
        goal_ = new_goal;
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::SetNextGoalAction>("SetNextGoal");
}
