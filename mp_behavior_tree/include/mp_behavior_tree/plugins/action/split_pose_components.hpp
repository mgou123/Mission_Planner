#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__SPLIT_POSE_COMPONENTS_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__SPLIT_POSE_COMPONENTS_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree
{
class SplitPoseComponents : public BT::SyncActionNode
{
public: 
    SplitPoseComponents(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf);
    
    ~SplitPoseComponents() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("pose", "Geometry pose"),
            BT::OutputPort<double>("x", "x value"),
            BT::OutputPort<double>("y", "y value"),
            BT::OutputPort<double>("z", "z value"),
            BT::OutputPort<double>("yaw", "yaw value")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif