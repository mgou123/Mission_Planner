// sets the parameters of the current task in the blackboard to be checked by other condition nodes

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_CURRENT_TASK_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_CURRENT_TASK_ACTION_HPP_

#include "ros/ros.h"
#include "vrx_gazebo/Task.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{
class UpdateCurrentTaskAction : public BT::ActionNodeBase
{
public:
    UpdateCurrentTaskAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    UpdateCurrentTaskAction() = delete;
    BT::NodeStatus tick() override;
    void halt() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("task_topic", "Task topic name"),
            BT::OutputPort<std::string>("current_task_name"),
            BT::OutputPort<std::string>("current_task_status")
        };
    }

private:
    void taskCallback(const vrx_gazebo::TaskConstPtr & msg);
    
    std::shared_ptr<ros::NodeHandle> node_;
    std::string task_topic_;
    ros::Subscriber task_sub_;

    std::string current_task_name_;
    std::string current_task_status_;

};

} // namespace behavior_tree

#endif