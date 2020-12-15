#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "asv_utils/geometry_utils.h"
#include "behavior_tree/plugins/action/order_and_set_waypoints_action.hpp"

namespace behavior_tree
{
OrderAndSetWaypointsAction::OrderAndSetWaypointsAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(action_name, conf)
{
    getInput("waypoints_topic", waypoints_topic_);
    
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");

    waypoints_sub_ = node_->subscribe(waypoints_topic_, 1, &OrderAndSetWaypointsAction::waypointsCallback, this);
}

BT::NodeStatus OrderAndSetWaypointsAction::tick() 
{
    if (!are_waypoints_set_) {
        return BT::NodeStatus::RUNNING;
    }

    if (first_time_ || are_waypoints_updated_) {
        if (!getInput("pose", robot_pose_)) {
            ROS_WARN("Pose is not available, waiting for pose...");
            return BT::NodeStatus::RUNNING;
        }

        ROS_INFO("Obtained new waypoints, ordering them...");

        geometry_utils::order_poses(robot_pose_, unordered_poses_, ordered_poses_);
        
        first_time_ = false;
        are_waypoints_updated_ = false;
    }

    ROS_INFO("Setting waypoints: ");

    for (auto pose : ordered_poses_) {
        std::cout << pose.position.x << " " << pose.position.y << std::endl;
    }

    setOutput("waypoints", ordered_poses_);

    return BT::NodeStatus::SUCCESS;
}

void OrderAndSetWaypointsAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

void OrderAndSetWaypointsAction::waypointsCallback(const asv_msgs::WaypointsConstPtr & msg)
{
    std::vector<geometry_msgs::Pose> new_poses;

    for (auto odom_wp : msg->waypoints) {
        new_poses.push_back(odom_wp.pose.pose);
    }

    if (first_time_) {
        unordered_poses_ = new_poses;
        are_waypoints_set_ = true;

    } else {
        if (!geometry_utils::is_equal(unordered_poses_, new_poses)) {
            unordered_poses_ = new_poses;
            are_waypoints_updated_ = true;
        }
    }
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::OrderAndSetWaypointsAction>("OrderAndSetWaypoints");
}
