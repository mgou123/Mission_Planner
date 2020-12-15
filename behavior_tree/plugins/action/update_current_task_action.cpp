#include "ros/ros.h"
#include "behavior_tree/plugins/action/update_current_task_action.hpp"

namespace behavior_tree
{
UpdateCurrentTaskAction::UpdateCurrentTaskAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(action_name, conf)
{
    getInput("task_topic", task_topic_);
    
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    task_sub_ = node_->subscribe(task_topic_, 1, &UpdateCurrentTaskAction::taskCallback, this);
}

BT::NodeStatus UpdateCurrentTaskAction::tick()
{
    if (current_task_name_.empty()) {
        return BT::NodeStatus::RUNNING;
    }

    setOutput("current_task_name", current_task_name_);
    setOutput("current_task_status", current_task_status_);

    return BT::NodeStatus::SUCCESS;
}

void UpdateCurrentTaskAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

void UpdateCurrentTaskAction::taskCallback(const vrx_gazebo::TaskConstPtr & msg) 
{
    current_task_name_ = msg->name;
    current_task_status_ = msg->state;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::UpdateCurrentTaskAction>("UpdateCurrentTask");
}