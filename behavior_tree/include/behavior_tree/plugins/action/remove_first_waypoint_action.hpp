// removes the first waypoint after it is completed to go to the second waypoint

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_FIRST_WAYPOINT_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_FIRST_WAYPOINT_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class RemoveFirstWaypointAction : public BT::SyncActionNode
{
public:
    RemoveFirstWaypointAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    RemoveFirstWaypointAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::BidirectionalPort<std::vector<geometry_msgs::Pose>>("waypoints", "Poses arranged in the order in which they should be visited")
        };
    }
};

} // namespace behavior_tree

#endif

