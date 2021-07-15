#include "mp_behavior_tree/plugins/action/set_relative_goal.hpp"
#include <tf2/LinearMath/Quaternion.h>
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

    geometry_msgs::PoseStamped relative_goal, current_pose, goal;
    double yaw, depth;
    if (!getInput("relative_goal", relative_goal)) {
        ROS_ERROR("[SetRelativeGoal] Unable to get relative goal!");
        return BT::NodeStatus::FAILURE;
    } else if (!getInput("global_pose", current_pose)) {
        ROS_ERROR("[SetRelativeGoal] Unable to get current pose!");
        return BT::NodeStatus::FAILURE;
    }
    
    getInput("global_frame", global_frame_);
    getInput("base_frame", base_frame_);

    //transform relative to global goal
    relative_goal.header.frame_id = base_frame_;
    relative_goal.header.stamp = ros::Time::now();

    relative_goal = tf_->transform(relative_goal, global_frame_, ros::Duration(transform_timeout_));
    
    goal.pose.position.x = current_pose.pose.position.x + relative_goal.pose.position.x;
    goal.pose.position.y = current_pose.pose.position.y + relative_goal.pose.position.y;
    goal.pose.position.z = current_pose.pose.position.z + relative_goal.pose.position.z;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(current_pose.pose.orientation, q_orig);
    tf2::convert(relative_goal.pose.orientation, q_rot);
    q_new = q_orig * q_rot;
    q_new.normalize();
    tf2::convert(q_new, goal.pose.orientation);

    setOutput("global_goal", goal);
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