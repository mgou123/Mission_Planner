#include "ros/ros.h"
#include <math.h>

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/action/publish_desired_heading_action.hpp"

namespace behavior_tree
{
PublishDesiredHeadingAction::PublishDesiredHeadingAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf),
    first_time_(true)
{
    getInput("big_spinny_topic", big_spinny_topic_);
    getInput("small_wriggle_topic", small_wriggle_topic_);
    getInput("hz", publish_rate_);

    publish_period_ = 1.0 / publish_rate_;

    double boundary_angle_deg;
    getInput("boundary_angle", boundary_angle_deg);
    boundary_angle_rad_ = boundary_angle_deg / 180 * M_PI;

    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    big_spinny_pub_ = node_->advertise<geometry_msgs::Quaternion>(big_spinny_topic_,1000);
    small_wriggle_pub_ = node_->advertise<geometry_msgs::Quaternion>(small_wriggle_topic_,1000);
}

BT::NodeStatus PublishDesiredHeadingAction::tick() 
{
    geometry_msgs::Quaternion desired_heading_quat;
    geometry_msgs::Pose robot_pose;

    double current_heading, desired_heading, yaw_difference;

    if (!getInput("desired_heading", desired_heading_quat)) {
        ROS_ERROR("[PublishDesiredHeading] Desired heading cannot be obtained from blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_ERROR("[PublishDesiredHeading] Cannot get robot pose");
        return BT::NodeStatus::FAILURE;
    }

    current_heading = geometry_utils::yawFromQuaternion(robot_pose.orientation);
    desired_heading = geometry_utils::yawFromQuaternion(desired_heading_quat);
    yaw_difference = abs(desired_heading - current_heading);

    if (first_time_) {
        first_time_ = false;
        last_published_time_ = ros::Time::now();

        if(yaw_difference >= boundary_angle_rad_) {
            big_spinny_pub_.publish(desired_heading_quat);
        } else {
            small_wriggle_pub_.publish(desired_heading_quat);
        }

        return BT::NodeStatus::SUCCESS;
    }

    if ((ros::Time::now() - last_published_time_).toSec() >= publish_period_) {

        if (yaw_difference >= boundary_angle_rad_) {
            big_spinny_pub_.publish(desired_heading_quat);
        } else {
            small_wriggle_pub_.publish(desired_heading_quat);
        }

        last_published_time_ = ros::Time::now();
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::PublishDesiredHeadingAction>("PublishDesiredHeading");
}
