#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__SET_BLACKBOARD_STRING_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__SET_BLACKBOARD_STRING_HPP_

#include <ros/ros.h>
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class SetBlackboardString : public BT::SyncActionNode
{
public:
    SetBlackboardString(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf);
    
    ~SetBlackboardString() {};

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<std::string>("key", "Key to set string to"),
            BT::InputPort<std::string>("value", "String value")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif