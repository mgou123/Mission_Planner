#include "ros/ros.h"
#include "behavior_tree/plugins/action/update_perception_results_action.hpp"

namespace behavior_tree
{
UpdatePerceptionResultsAction::UpdatePerceptionResultsAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(action_name, conf)
{
    getInput("detected_topic", detected_topic_);
    
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    detected_sub_ = node_->subscribe(detected_topic_, 1, &UpdatePerceptionResultsAction::detectedCallback, this);
}

BT::NodeStatus UpdatePerceptionResultsAction::tick()
{
    if (detected_objects_.empty()) {
        return BT::NodeStatus::RUNNING;
    }

    setOutput("detected_objects", detected_objects_);

    return BT::NodeStatus::SUCCESS;
}

void UpdatePerceptionResultsAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

void UpdatePerceptionResultsAction::detectedCallback(const asv_msgs::DetectedObjectsConstPtr & msg) 
{
    detected_objects_ = msg->detected;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::UpdatePerceptionResultsAction>("UpdatePerceptionResults");
}