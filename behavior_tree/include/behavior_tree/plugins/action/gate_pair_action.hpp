// action for ASV to take if only pairs of gates are detected, ASV heads to th

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__GATE_PAIR_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__GATE_PAIR_ACTION_HPP_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class GatePairAction : public BT::SyncActionNode
{
public:
    GatePairAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    GatePairAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("gate_1", "RedTotem pose"),
            BT::InputPort<geometry_msgs::PoseStamped>("gate_2", "NonRedTotem pose"),
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::OutputPort<geometry_msgs::Quaternion>("desired_heading", "Heading for asv to go towards")
        };
    }
};



} // namespace behavior_tree

#endif


