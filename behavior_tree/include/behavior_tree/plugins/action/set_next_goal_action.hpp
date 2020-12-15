// sets first goal in waypoints to blackboard

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__SET_NEXT_GOAL_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__SET_NEXT_GOAL_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class SetNextGoalAction : public BT::SyncActionNode
{
public:
    SetNextGoalAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    SetNextGoalAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<geometry_msgs::Pose>>("waypoints", "Poses arranged in the order in which they should be visited"),
            BT::OutputPort<geometry_msgs::Pose>("goal", "Pose goal")
        };
    }
private:
    geometry_msgs::Pose goal_;
    bool first_time_{false};
};

} // namespace behavior_tree

#endif

