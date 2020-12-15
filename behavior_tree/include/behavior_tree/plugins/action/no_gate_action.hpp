// action for ASV to take if no gates are detected, ASV heads straight

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__NO_GATE_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__NO_GATE_ACTION_HPP_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class NoGateAction : public BT::SyncActionNode
{
public:
    NoGateAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    NoGateAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::OutputPort<geometry_msgs::Quaternion>("desired_heading", "Heading for asv to go towards")
        };
    }
};

} // namespace behavior_tree

#endif

