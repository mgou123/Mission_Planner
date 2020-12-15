// holds the current position, currently does nothing as that should be the desired outcome
// can expand behaviour in the future

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__HOLD_POSITION_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__HOLD_POSITION_ACTION_HPP_

#include "ros/ros.h"
#include "behaviortree_cpp_v3/action_node.h"


namespace behavior_tree
{

class HoldPositionAction : public BT::ActionNodeBase
{
public:
    HoldPositionAction(
        const std::string & name,
        const BT::NodeConfiguration & conf);
    
    HoldPositionAction() = delete;

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override;

    void halt() override;
};

} //namespace behavior_tree

#endif