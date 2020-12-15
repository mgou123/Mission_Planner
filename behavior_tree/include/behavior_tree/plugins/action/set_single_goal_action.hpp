// sets single goal from topic

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__SET_SINGLE_GOAL_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__SET_SINGLE_GOAL_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class SetSingleGoalAction : public BT::ActionNodeBase
{
public:
    SetSingleGoalAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    SetSingleGoalAction() = delete;

    BT::NodeStatus tick() override;

    void halt() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal_topic", "Topic for goal"),
            BT::OutputPort<geometry_msgs::Pose>("goal", "Pose goal")
        };
    }

private:
    void goalCallback(const nav_msgs::OdometryConstPtr & msg);

    std::shared_ptr<ros::NodeHandle> node_;
    std::string goal_topic_;
    ros::Subscriber goal_sub_;
    geometry_msgs::Pose goal_;
    bool first_time_{true};
    bool is_goal_set_{false};
    bool is_new_goal_{false};
};

} // namespace behavior_tree

#endif

