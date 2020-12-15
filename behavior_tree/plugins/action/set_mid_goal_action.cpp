#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/action/set_mid_goal_action.hpp"

namespace behavior_tree
{
SetMidGoalAction::SetMidGoalAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf),
    clearance_(50.0)
{
    getInput("clearance", clearance_);
}

BT::NodeStatus SetMidGoalAction::tick() 
{
    
    if (first_time_ || config().blackboard->get<bool>("goal_updated")) {
        if (!getInput("goal", goal_)) {
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("pose", robot_pose_)) {
            return BT::NodeStatus::FAILURE;
        }
        
        first_time_ = false;
        setOutput("mid_goal", getMidGoal());
        
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::SUCCESS;
}

geometry_msgs::Pose SetMidGoalAction::getMidGoal() 
{
    geometry_msgs::Pose mid_pose;

    ROS_INFO("Trying to calculate mid_goal...");
    ROS_INFO("Current goal: [%f, %f, %f]", goal_.position.x, goal_.position.y, goal_.position.z);
    ROS_INFO("Current pose: [%f, %f, %f]", robot_pose_.position.x, robot_pose_.position.y, robot_pose_.position.z);

    double midpt_x = (robot_pose_.position.x - goal_.position.x) / 2 + goal_.position.x;
    double midpt_y = (robot_pose_.position.y - goal_.position.y) / 2 + goal_.position.y;
    double g = (robot_pose_.position.y - goal_.position.y) / (robot_pose_.position.x - goal_.position.x);
    g = -1 / g; // tan(theta)
    mid_pose.position.x = midpt_x + clearance_ * (1 / sqrt(1 + g * g)); // mid_x + d cos(theta)
    mid_pose.position.y = midpt_y + clearance_ * (g / sqrt(1 + g * g)); // mid_y + d sin(theta)
    
    mid_pose.position.z = goal_.position.z;
    mid_pose.orientation = goal_.orientation;

    ROS_INFO("Obtained mid pose:");
    ROS_INFO("Mid goal: [%f, %f, %f]", mid_pose.position.x, mid_pose.position.y, mid_pose.position.z);

    return mid_pose;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::SetMidGoalAction>("SetMidGoal");
}
