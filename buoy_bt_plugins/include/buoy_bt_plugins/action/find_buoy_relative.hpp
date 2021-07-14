#ifndef BUOY_BT_PLUGINS__PLUGINS__ACTION__FIND_BUOY_RELATIVE_HPP_
#define BUOY_BT_PLUGINS__PLUGINS__ACTION__FIND_BUOY_RELATIVE_HPP_

#include <ros/ros.h>
#include "vision/DetectedObjects.h"
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree
{
class FindBuoyRelative : public BT::SyncActionNode {
public :
    FindBuoyRelative(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);

    ~FindBuoyRelative() {}

    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<vision::DetectedObjects>("vision_objects", "Objects detected by vision pipeline as a vision::DetectedObjects message"),
            BT::InputPort<std::string>("target_identity", "Target to be made as goal. String"),
            BT::OutputPort<float>("forward", "Forward as float"),
            BT::OutputPort<float>("sideways", "Sideways as float"),
            BT::OutputPort<float>("depth", "Depth as float"),
            BT::OutputPort<float>("yaw", "Yaw as float"),
        };
    }

    BT::NodeStatus tick() override;
};
} // namespace mp_behavior_tree

#endif