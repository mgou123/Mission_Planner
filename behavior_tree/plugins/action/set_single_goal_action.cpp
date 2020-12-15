#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/action/set_single_goal_action.hpp"

namespace behavior_tree
{
SetSingleGoalAction::SetSingleGoalAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(action_name, conf)
{
    getInput("goal_topic", goal_topic_);
    
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");

    goal_sub_ = node_->subscribe(goal_topic_, 1, &SetSingleGoalAction::goalCallback, this);
}

BT::NodeStatus SetSingleGoalAction::tick() 
{
    if (!is_goal_set_) {
        return BT::NodeStatus::RUNNING;
    }

    setOutput("goal", goal_);

    if (first_time_) {
        first_time_ = false;
    } else {
        if (is_new_goal_) {
            config().blackboard->set("goal_updated", true);
            is_new_goal_ = false;
        }
    }

    return BT::NodeStatus::SUCCESS;
}

void SetSingleGoalAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

void SetSingleGoalAction::goalCallback(const nav_msgs::OdometryConstPtr & msg)
{
    if (is_goal_set_) {
        if (!geometry_utils::is_equal(goal_, msg->pose.pose)) {
            is_new_goal_ = true;
            goal_ = msg->pose.pose;
        }
    } else {
        is_goal_set_ = true;
        goal_ = msg->pose.pose;
    }
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::SetSingleGoalAction>("SetSingleGoal");
}
