#include <string>
#include <vector>
#include "mp_behavior_tree/plugins/condition/is_pose_updated_condition.hpp"

namespace mp_behavior_tree
{

IsPoseUpdatedCondition::IsPoseUpdatedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

IsPoseUpdatedCondition::~IsPoseUpdatedCondition() {
    cleanup();
}

BT::NodeStatus IsPoseUpdatedCondition::tick() {
    if (status() == BT::NodeStatus::IDLE) {
        config().blackboard->get<geometry_msgs::Pose>("pose", pose_);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::Pose current_pose;
    config().blackboard->get<geometry_msgs::Pose>("pose", current_pose);

    if (pose_ != current_pose) {
        pose_ = current_pose;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

} //namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsPoseUpdatedCondition>("IsPoseUpdated");
}