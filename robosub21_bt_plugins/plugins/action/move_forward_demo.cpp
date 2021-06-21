#include "robosub21_bt_plugins/action/move_forward_demo.hpp"

namespace mp_behavior_tree
{
MoveForwardDemo::MoveForwardDemo(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf)
: BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus MoveForwardDemo::tick() 
{
    nav_msgs::Odometry odom_pose;
    geometry_msgs::PoseStamped goal;
    double distance;
    if (!getInput("pose", odom_pose)) {
        ROS_ERROR("[MoveForwardDemo] Unable to obtain pose from blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("distance", distance)) {
        distance = 5.0;
    }

    goal.pose = odom_pose.pose.pose;
    goal.pose.position.x = goal.pose.position.x + distance;
    
    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;

}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mp_behavior_tree::MoveForwardDemo>("MoveForwardDemo");
}
