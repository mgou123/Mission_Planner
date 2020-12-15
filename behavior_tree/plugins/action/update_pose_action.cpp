#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "behavior_tree/plugins/action/update_pose_action.hpp"

namespace behavior_tree 
{
UpdatePoseAction::UpdatePoseAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(action_name, conf) 
{
    getInput("odom_topic", odom_topic_);

    last_pose_update_ = ros::Time(0);
    setOutput("last_pose_update", last_pose_update_);

    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    odom_sub_ = node_->subscribe(odom_topic_, 1, &UpdatePoseAction::odomCallback, this);

}

BT::NodeStatus UpdatePoseAction::tick()
{
    if (last_pose_update_ == ros::Time(0)) {
        ROS_WARN("/clock not published yet, time is 0. Or odom not being published.");
        return BT::NodeStatus::RUNNING;
    }

    setOutput("pose", robot_pose_);
    setOutput("last_pose_update", last_pose_update_);
    
    return BT::NodeStatus::SUCCESS;
}

void UpdatePoseAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

void UpdatePoseAction::odomCallback(const nav_msgs::OdometryConstPtr & msg)
{
    robot_pose_ = msg->pose.pose;
    last_pose_update_ = ros::Time::now();
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::UpdatePoseAction>("UpdatePose");
}