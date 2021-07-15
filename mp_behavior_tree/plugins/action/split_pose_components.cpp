#include "mp_behavior_tree/plugins/action/split_pose_components.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mp_behavior_tree
{
SplitPoseComponents::SplitPoseComponents(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration &conf) 
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus SplitPoseComponents::tick() {
    geometry_msgs::PoseStamped pose;

    if (!getInput("pose", pose)) {
        ROS_WARN("[SplitPoseComponents] Cannot get pose!");
        return BT::NodeStatus::FAILURE;
    }

    tf2::Quaternion quat;
    double roll, pitch, yaw;

    tf2::convert(pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    yaw = yaw / M_PI * 180.0;

    if (yaw < 0) {
      yaw += 360.0;
    }

    std::cout << "Yaw: " << yaw << std::endl; 

    setOutput("x", pose.pose.position.x);
    setOutput("y", pose.pose.position.y);
    setOutput("z", pose.pose.position.z);
    
    setOutput("yaw", yaw);

    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SplitPoseComponents>("SplitPoseComponents");
}