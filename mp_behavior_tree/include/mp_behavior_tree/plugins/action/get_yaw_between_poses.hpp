#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__GET_YAW_BETWEEN_POSES_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__GET_YAW_BETWEEN_POSES_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree
{
class GetYawBetweenPoses : public BT::SyncActionNode
{
public: 
    GetYawBetweenPoses(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf);
    
    ~GetYawBetweenPoses() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("source_pose", "Initial starting pose"),
            BT::InputPort<geometry_msgs::PoseStamped>("target_pose", "Final pose"),
            BT::OutputPort<double>("relative_yaw", "Relative yaw to turn to the target pose from the source pose"),
            BT::OutputPort<double>("absolute_yaw", "Absolute yaw to turn to the target pose from the source pose")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif