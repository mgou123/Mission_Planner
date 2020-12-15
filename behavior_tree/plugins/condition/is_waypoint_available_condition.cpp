#include "behavior_tree/plugins/condition/is_waypoint_available_condition.hpp"

namespace behavior_tree
{

IsWaypointAvailableCondition::IsWaypointAvailableCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
{
}

BT::NodeStatus IsWaypointAvailableCondition::tick()
{
    std::vector<geometry_msgs::Pose> waypoints;

    if (!getInput("waypoints", waypoints)) {
        return BT::NodeStatus::FAILURE;
    }

    if (waypoints.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::IsWaypointAvailableCondition>("IsWaypointAvailable");
}