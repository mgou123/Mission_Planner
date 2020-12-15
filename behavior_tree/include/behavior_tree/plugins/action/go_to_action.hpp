// uses asv path planner to go to the goal pose (in the blackboard)

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__GO_TO_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__GO_TO_ACTION_HPP_

#include <ros/ros.h>

#include <path_planner/TargetPositionAction.h>
#include "behavior_tree/bt_action_node.hpp"

namespace behavior_tree
{

class GoToAction : public BtActionNode<path_planner::TargetPositionAction,
                                       path_planner::TargetPositionGoal,
                                       path_planner::TargetPositionResult>
{
public:
    GoToAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;
    void on_wait_for_result() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
            {
                BT::InputPort<geometry_msgs::Pose>("goal", "Destination goal")
            });
    }

private:
    bool first_time_{true};
};

} // namespace behavior_tree

#endif