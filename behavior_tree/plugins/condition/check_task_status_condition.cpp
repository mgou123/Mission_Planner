#include "behavior_tree/plugins/condition/check_task_status_condition.hpp"

namespace behavior_tree
{

CheckTaskStatusCondition::CheckTaskStatusCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
{
    getInput("task_status", task_status_);
}

BT::NodeStatus CheckTaskStatusCondition::tick()
{
    std::string current_task_status;

    if (!getInput("current_task_status", current_task_status)) {
        return BT::NodeStatus::FAILURE;
    }

    if (current_task_status == task_status_) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

} // namespace behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::CheckTaskStatusCondition>("CheckTaskStatus");
}