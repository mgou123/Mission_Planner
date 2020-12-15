// sets single goal from topic

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__SET_MID_GOAL_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__SET_MID_GOAL_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class SetMidGoalAction : public BT::SyncActionNode
{
public:
    SetMidGoalAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    SetMidGoalAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::Pose>("goal", "latest goal"),
            BT::InputPort<geometry_msgs::Pose>("pose", "robot pose"),
            BT::InputPort<double>("clearance", 50.0, "Distance from goal to place mid goal"),
            BT::OutputPort<geometry_msgs::Pose>("mid_goal", "intermediate pose goal")
        };
    }

private:
    geometry_msgs::Pose getMidGoal();

    geometry_msgs::Pose goal_;
    geometry_msgs::Pose robot_pose_;
    double clearance_;

    bool first_time_{true};
};

} // namespace behavior_tree

#endif

