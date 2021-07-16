#include "mp_behavior_tree/plugins/action/set_relative_goal.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mp_behavior_tree
{

SetRelativeGoal::SetRelativeGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus SetRelativeGoal::tick() {
    if (!initialized_) {
      initialize();
    }

    geometry_msgs::PoseStamped relative_goal, pose, goal;
    double yaw, depth;
    if (!getInput("relative_goal", relative_goal)) {
        ROS_ERROR("[SetRelativeGoal] Unable to get relative goal!");
        return BT::NodeStatus::FAILURE;
    } else if (!getInput("pose", pose)) {
        ROS_ERROR("[SetRelativeGoal] Unable to get current pose!");
        return BT::NodeStatus::FAILURE;
    }


    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;

    transform.transform.rotation = pose.pose.orientation;
    

    std::cout << "Transform: " << transform << std::endl;
    std::cout << "Relative goal: " << relative_goal << std::endl;

    tf2::doTransform(relative_goal, goal, transform); 

    std::cout << "Transformed goal: " << goal << std::endl;

    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
}

void SetRelativeGoal::initialize() {
  node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  
  if (node_->hasParam("transform_timeout")) {
    node_->getParam("transform_timeout", transform_timeout_);
  } else {
    transform_timeout_ = 1.0;
  }

  initialized_ = true;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SetRelativeGoal>("SetRelativeGoal");
}