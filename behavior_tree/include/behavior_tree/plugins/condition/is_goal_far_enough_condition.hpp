// checks if desired position and orientation goal is reached

#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_FAR_ENOUGH_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_FAR_ENOUGH_CONDITION_HPP_

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

#include "behaviortree_cpp_v3/condition_node.h"

namespace behavior_tree
{

class IsGoalFarEnoughCondition : public BT::ConditionNode
{
public:
    IsGoalFarEnoughCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    IsGoalFarEnoughCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::InputPort<geometry_msgs::Pose>("goal", "Pose goal"),
            BT::InputPort<double>("clearance", 30.0, "Minimum distance to goal"),
        };
    }

private:    
    double clearance_;
};

} // namepsace behavior_tree

#endif