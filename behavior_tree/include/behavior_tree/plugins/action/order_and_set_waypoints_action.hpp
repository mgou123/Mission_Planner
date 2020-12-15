// finds the shortest path through all the waypoints and sets them in the board

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__ORDER_AND_SET_WAYPOINTS_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__ORDER_AND_SET_WAYPOINTS_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "asv_msgs/Waypoints.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class OrderAndSetWaypointsAction : public BT::ActionNodeBase
{
public:
    OrderAndSetWaypointsAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    OrderAndSetWaypointsAction() = delete;

    BT::NodeStatus tick() override;

    void halt() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("waypoints_topic", "Topic for waypoints"),
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest pose"),
            BT::OutputPort<std::vector<geometry_msgs::Pose>>("waypoints", "Poses arranged in the order in which they should be visited")
        };
    }

private:
    void waypointsCallback(const asv_msgs::WaypointsConstPtr & msg);

    std::shared_ptr<ros::NodeHandle> node_;
    std::string waypoints_topic_;
    ros::Subscriber waypoints_sub_;
    geometry_msgs::Pose robot_pose_;

    std::vector<geometry_msgs::Pose> ordered_poses_;
    std::vector<geometry_msgs::Pose> unordered_poses_;
    
    bool are_waypoints_set_{false};
    bool are_waypoints_updated_{false};

    bool first_time_{true};
};

} // namespace behavior_tree

#endif

