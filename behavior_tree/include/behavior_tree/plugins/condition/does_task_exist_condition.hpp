// checks if a certain task exists, name of task is provided through input port

#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__DOES_TASK_EXIST_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__DOES_TASK_EXIST_CONDITION_HPP_

#include "ros/ros.h"

#include "behaviortree_cpp_v3/condition_node.h"

namespace behavior_tree 
{

class DoesTaskExistCondition : public BT::ConditionNode
{
public:
    DoesTaskExistCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    DoesTaskExistCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<std::string>("current_task_name", "Current task name"),
            BT::InputPort<std::string>("task_name", "Desired task name")
        };
    }
private:
    std::string task_name_;
};

} // namespace behavior_tree

#endif