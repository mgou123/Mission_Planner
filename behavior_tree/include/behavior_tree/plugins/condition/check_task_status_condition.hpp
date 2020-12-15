// checks if the current task is of a certain status

#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TASK_STATUS_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_TASK_STATUS_CONDITION_HPP_

#include "ros/ros.h"

#include "behaviortree_cpp_v3/condition_node.h"

namespace behavior_tree
{

class CheckTaskStatusCondition : public BT::ConditionNode
{
public:
    CheckTaskStatusCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    CheckTaskStatusCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("current_task_status", "Current status of task"),
            BT::InputPort<std::string>("task_status", "Desired task status")
        };
    }

private:
    std::string task_status_;
};

} // namespace behavior_tree

#endif