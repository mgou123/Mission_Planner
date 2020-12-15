#include "behavior_tree/plugins/condition/does_task_exist_condition.hpp"

namespace behavior_tree
{

DoesTaskExistCondition::DoesTaskExistCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
    getInput("task_name", task_name_);
}

BT::NodeStatus DoesTaskExistCondition::tick()
{
    std::string current_task_name;

    if (!getInput("current_task_name", current_task_name)) {
        return BT::NodeStatus::FAILURE;
    }

    if (current_task_name == task_name_) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::DoesTaskExistCondition>("DoesTaskExist");
}
