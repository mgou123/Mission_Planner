#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__SET_RELATIVE_GOAL_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__SET_RELATIVE_GOAL_HPP_

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace mp_behavior_tree
{
class SetRelativeGoal : public BT::SyncActionNode
{
public:
    SetRelativeGoal(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);

    ~SetRelativeGoal() {}

    void initialize();
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("relative_goal", "Relative pose goal (refer to bt conversions for specification)"),
            BT::InputPort<geometry_msgs::PoseStamped>("global_pose", "Current pose"),
            BT::InputPort<std::string>("global_frame", std::string("world"), "Global frame"),
            BT::InputPort<std::string>("base_frame", std::string("base_link"), "Robot base frame"),
            BT::OutputPort<geometry_msgs::PoseStamped>("global_goal", "Port to set goal to"),
        };
    }

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<ros::NodeHandle> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;

    bool initialized_;
    std::string global_frame_;
    std::string base_frame_;

    double transform_timeout_;

};

} // namespace mp_behavior_tree



#endif