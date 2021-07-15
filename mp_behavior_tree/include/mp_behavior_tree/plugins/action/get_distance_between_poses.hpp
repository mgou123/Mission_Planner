#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__GET_DISTANCE_BETWEEN_POSES_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__GET_DISTANCE_BETWEEN_POSES_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree
{
class GetDistanceBetweenPoses : public BT::SyncActionNode
{
public: 
    GetDistanceBetweenPoses(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf);
    
    ~GetDistanceBetweenPoses() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("source_pose", "Initial starting pose"),
            BT::InputPort<geometry_msgs::PoseStamped>("target_pose", "Final pose"),
            BT::OutputPort<geometry_msgs::PoseStamped>("relative_pose", "Distance between poses in a geometry pose message (x value)"),
            BT::OutputPort<double>("distance", "Distance between poses")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif