// checks if desired position and orientation goal is reached

#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

#include "behaviortree_cpp_v3/condition_node.h"

namespace behavior_tree
{

class IsGoalReachedCondition : public BT::ConditionNode
{
public:
    IsGoalReachedCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    IsGoalReachedCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::InputPort<geometry_msgs::Pose>("goal", "Pose goal"),
            BT::InputPort<ros::Time>("last_pose_update", "timestamp of latest pose update"),
            BT::InputPort<double>("max_error", 0.1, "Maximum error from goal"),
            BT::InputPort<double>("duration_tol", 1.0, "Tolerance for time since last odom update (seconds)")
        };
    }

private:    
    double max_error_;
    double dur_tol_;
};

} // namepsace behavior_tree

#endif