#include "behavior_tree/plugins/action/hold_position_action.hpp"

namespace behavior_tree
{

HoldPositionAction::HoldPositionAction(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(name, conf)
{
}

BT::NodeStatus HoldPositionAction::tick()
{
    return BT::NodeStatus::SUCCESS;
}

void HoldPositionAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

} //namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) 
{
    factory.registerNodeType<behavior_tree::HoldPositionAction>("HoldPosition");
}