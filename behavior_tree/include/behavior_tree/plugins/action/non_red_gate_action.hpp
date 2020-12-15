// action for ASV to take if only the non-red gates are detected, ASV heads to th

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__NON_RED_GATE_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__NON_RED_GATE_ACTION_HPP_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class NonRedGateAction : public BT::SyncActionNode
{
public:
    NonRedGateAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    NonRedGateAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("offset_distance", "distance to offset (in m)" ),
            BT::InputPort<geometry_msgs::PoseStamped>("gate_1", "Closest gate to asv"),
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::OutputPort<geometry_msgs::Quaternion>("desired_heading", "Heading for asv to go towards")
        };
    }

private:
    double offset_distance_;
};



} // namespace behavior_tree

#endif

