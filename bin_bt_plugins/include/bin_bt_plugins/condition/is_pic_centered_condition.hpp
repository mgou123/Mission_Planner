#ifndef BIN_BT_PLUGINS__CONDITION__IS_PIC_CENTERED_CONDITION_
#define BIN_BT_PLUGINS__CONDITION__IS_PIC_CENTERED_CONDITION_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsPicCenteredCondition : public BT::ConditionNode 
{
public:
    IsPicCenteredCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsPicCenteredCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("detected_objects", "Detected Objects"),
            BT::InputPort<std::string>("pic_identifier", "Which pic to go"),
            BT::InputPort<float>("center_offset_x", "allowed offset to center on x-axis"),
            BT::InputPort<float>("center_offset_y", "allowed offset to center on y-axis")
        };
    }
};

} // namespace mp_behavior_tree

#endif 
