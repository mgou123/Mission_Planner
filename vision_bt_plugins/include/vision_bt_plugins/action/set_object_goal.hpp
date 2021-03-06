#ifndef VISION_BT_PLUGINS__ACTION__SET_OBJECT_GOAL_HPP_
#define VISION_BT_PLUGINS__ACTION__SET_OBJECT_GOAL_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

#include "bb_msgs/DetectedObjects.h"

namespace mp_behavior_tree
{
class SetObjectGoal : public BT::SyncActionNode 
{
public:
    SetObjectGoal(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);

    ~SetObjectGoal() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<bb_msgs::DetectedObjects>("vision_objects", "Objects detected by vision pipeline as a bb_msgs::DetectedObjects message"),
            BT::InputPort<std::string>("target_identity", "Target to be made as goal. String"),
            BT::OutputPort<geometry_msgs::PoseStamped>("relative_goal", "Goal to move to"),
            BT::OutputPort<double>("absolute_depth", "Depth of object"),
            BT::OutputPort<double>("relative_yaw", "Yaw of object"),
        };
    }

    BT::NodeStatus tick() override;
};
} // namespace mp_behavior_tree

#endif
