// updates odometry from topic, checks for timeout

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_POSE_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_POSE_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class UpdatePoseAction : public BT::ActionNodeBase
{
public:
    UpdatePoseAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    UpdatePoseAction() = delete;

    BT::NodeStatus tick() override;
    
    void halt() override;
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("odom_topic", "Topic for robot odom"),
            BT::OutputPort<geometry_msgs::Pose>("pose", "Robot pose"),
            BT::OutputPort<ros::Time>("last_pose_update", "timestamp for latest pose update")
        };
    }

private:
    void odomCallback(const nav_msgs::OdometryConstPtr & msg);
    
    std::shared_ptr<ros::NodeHandle> node_;
    ros::Subscriber odom_sub_;
    std::string odom_topic_;
    geometry_msgs::Pose robot_pose_;

    ros::Time last_pose_update_;

};

} // namespace behavior_tree

#endif 