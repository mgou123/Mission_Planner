#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__CONVERT_ODOM_TO_POSE_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__CONVERT_ODOM_TO_POSE_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree
{
class ConvertOdomToPose : public BT::SyncActionNode
{
public: 
    ConvertOdomToPose(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf);
    
    ~ConvertOdomToPose() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<nav_msgs::Odometry>("odom_pose", "Pose as a nav_msgs::Odometry message"),
            BT::OutputPort<geometry_msgs::PoseStamped>("pose", "Pose as a geometry_msgs::PoseStamped message")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif