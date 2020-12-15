// publish perception results for VORC perception task

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_DESIRED_HEADING_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_DESIRED_HEADING_ACTION_HPP_

#include <ros/ros.h>

#include "geometry_msgs/Pose.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class PublishDesiredHeadingAction : public BT::SyncActionNode
{
public:
    PublishDesiredHeadingAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    PublishDesiredHeadingAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("big_spinny_topic", "Topic for big spinning movement"),
            BT::InputPort<std::string>("small_wriggle_topic", "Topic for small turning movement"),
            BT::InputPort<double>("boundary_angle" , " > Boundary angle = big spinny (in degrees)"),
            BT::InputPort<double>("hz", 1.0, "Publish rate"),
            
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::InputPort<geometry_msgs::Quaternion>("desired_heading", "Heading for asv to go towards")
        };
    }

private:
    std::shared_ptr<ros::NodeHandle> node_;
    std::string big_spinny_topic_;
    std::string small_wriggle_topic_;

    ros::Publisher big_spinny_pub_;
    ros::Publisher small_wriggle_pub_;

    double boundary_angle_rad_;

    double publish_rate_;
    double publish_period_;
    ros::Time last_published_time_;

    bool first_time_;

};



} // namespace behavior_tree

#endif

