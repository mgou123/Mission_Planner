#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__SET_RELATIVE_GOAL_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__SET_RELATIVE_GOAL_HPP_

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/PoseStamped.h>


namespace mp_behavior_tree
{
class SetRelativeGoal : public BT::SyncActionNode
{
public:
    SetRelativeGoal(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);

    ~SetRelativeGoal() {}
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("relative_goal", "Relative pose goal (refer to bt conversions for specification)"),
            BT::InputPort<geometry_msgs::PoseStamped>("pose", "Current pose"),
            BT::InputPort<double>("yaw_lock", "Yaw to lock to. If specified, relative yaw in pose will be ignored."),
            BT::InputPort<double>("depth_lock", "Depth to lock to. If specified, relative depth in pose will be ignored."),
            BT::OutputPort<geometry_msgs::PoseStamped>("goal", "Port to set goal to")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree



#endif