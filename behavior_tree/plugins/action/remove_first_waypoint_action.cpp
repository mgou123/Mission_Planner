#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/action/remove_first_waypoint_action.hpp"

namespace behavior_tree
{
RemoveFirstWaypointAction::RemoveFirstWaypointAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{
}

BT::NodeStatus RemoveFirstWaypointAction::tick() 
{
    std::vector<geometry_msgs::Pose> waypoints;

    if (!getInput("waypoints", waypoints)) {
        ROS_ERROR("Unable to obtain waypoints from blackboard in remove_first_waypoint action");
        return BT::NodeStatus::FAILURE;
    }

    if (waypoints.empty()) {
        ROS_WARN("No waypoints available!");
        return BT::NodeStatus::FAILURE;
    }

    waypoints.erase(waypoints.begin());

    setOutput("waypoints", waypoints);

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::RemoveFirstWaypointAction>("RemoveFirstWaypoint");
}
