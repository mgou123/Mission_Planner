// checks if there are any waypoints left to reach

#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WAYPOINT_AVAILABLE_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WAYPOINT_AVAILABLE_CONDITION_HPP_

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

#include "behaviortree_cpp_v3/condition_node.h"

namespace behavior_tree 
{

class IsWaypointAvailableCondition : public BT::ConditionNode
{
public:
    IsWaypointAvailableCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    IsWaypointAvailableCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<std::vector<geometry_msgs::Pose>>("waypoints", "Poses arranged in the order in which they should be visited"),
        };
    }
};

} // namespace behavior_tree

#endif