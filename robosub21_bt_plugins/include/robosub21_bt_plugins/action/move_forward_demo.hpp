#ifndef ROBOSUB21_BT_PLUGINS__ACTION__MOVE_FORWARD_DEMO_HPP_
#define ROBOSUB21_BT_PLUGINS__ACTION__MOVE_FORWARD_DEMO_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree
{
class MoveForwardDemo : public BT::SyncActionNode
{
public:
    MoveForwardDemo(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf);

    ~MoveForwardDemo() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("distance", "Distance to move forward, default to 5m"),
            BT::InputPort<nav_msgs::Odometry>("pose", "Current pose"),
            BT::OutputPort<geometry_msgs::PoseStamped>("goal", "Goal set 5m forward")
        };
    }
};

}

#endif