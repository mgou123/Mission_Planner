#ifndef BIN_BT_PLUGINS__ACTION__SET_ADJUSTMENT_HPP_
#define BIN_BT_PLUGINS__ACTION__SET_ADJUSTMENT_HPP_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class SetAdjustment : public BT::SyncActionNode
{
public:
    SetAdjustment(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~SetAdjustment() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("detected_objects", "Detected Objects"),
            BT::InputPort<float>("param_x", "x unit movement"),
            BT::InputPort<float>("param_y", "x unit movement"),
            BT::OutputPort<float>("x_goal", "sideways move"),
            BT::OutputPort<float>("y_goal", "forward move"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 
