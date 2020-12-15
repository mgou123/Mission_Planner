#include "behavior_tree/plugins/action/go_to_action.hpp"

namespace behavior_tree
{

GoToAction::GoToAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<path_planner::TargetPositionAction,
                 path_planner::TargetPositionGoal,
                 path_planner::TargetPositionResult>(xml_tag_name, action_name, conf)
{
    config().blackboard->set("goal_updated", false);
}

void GoToAction::on_tick()
{
    getInput("goal", goal_.target.pose);
    ROS_INFO("Current goal: [%f, %f, %f]", goal_.target.pose.position.x, goal_.target.pose.position.y, goal_.target.pose.position.z);
    first_time_ = true;
}

void GoToAction::on_wait_for_result()
{
    // check if goal has been updated
    if (config().blackboard->get<bool>("goal_updated")) {
        // reset flag in blackboard
        config().blackboard->set("goal_updated", false);

        // if it's the initial goal and not a new goal, return without updating the goal
        if (first_time_) {
            first_time_ = false;
            return;
        }

        // get the new goal and set the flag so that we send the new goal to the action server on the next loop iteration
        getInput("goal", goal_.target);
        goal_updated_ = true;
    }
}

} // namespace behavior_tree

//create custom builder for registering node with ros action name
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = 
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<behavior_tree::GoToAction>(
                name, "/asv/PathPlanner", config);
        };
    
    factory.registerBuilder<behavior_tree::GoToAction>("GoTo", builder);
}