#ifndef VISION_BT_PLUGINS__ACTION__COMPUTE_GOAL_HPP_
#define VISION_BT_PLUGINS__ACTION__COMPUTE_GOAL_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

#include "vision/DetectedObjects.h"

namespace mp_behavior_tree
{
class ComputeGoal : public BT::SyncActionNode 
{
public:
    ComputeGoal(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);

    ~ComputeGoal() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<vision::DetectedObjects>("vision_objects", "Objects detected by vision pipeline as a vision::DetectedObjects message"),
            BT::InputPort<std::string>("target_identity", "Target to be made as goal. String"),
            BT::InputPort<std::string>("main_target", "Main target for which range is extracted"),
            BT::OutputPort<geometry_msgs::PoseStamped>("goal", "Goal to move to"),
            BT::OutputPort<double>("absolute_depth", "Depth of object"),
            BT::OutputPort<double>("absolute_yaw", "Yaw of object"),
        };
    }

    BT::NodeStatus tick() override;
};
} // namespace mp_behavior_tree

#endif
