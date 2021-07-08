#include "mp_behavior_tree/plugins/action/set_blackboard_string.hpp"

namespace mp_behavior_tree
{
SetBlackboardString::SetBlackboardString(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus SetBlackboardString::tick() {
    std::string key, value;
    if (!getInput("key", key)) {
        ROS_ERROR("[SetBlackboardString] Unable to get key!");
        return BT::NodeStatus::FAILURE;
    } else if (!getInput("value", value)) {
        ROS_ERROR("[SetBackboardString] Unable to get value!");
        return BT::NodeStatus::FAILURE;
    } else {
        config().blackboard->set<std::string>(key, value);
        return BT::NodeStatus::SUCCESS;
    }
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SetBlackboardString>("SetBlackboardString");
}