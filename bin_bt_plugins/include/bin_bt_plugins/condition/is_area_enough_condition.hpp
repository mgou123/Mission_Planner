#ifndef BIN_BT_PLUGINS__CONDITION__IS_AREA_ENOUGH_CONDITION_
#define BIN_BT_PLUGINS__CONDITION__IS_AREA_ENOUGH_CONDITION_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "bb_msgs/DetectedObjects.h"
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
            BT::InputPort<bb_msgs::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<std::string>("pic_identifier", "Which pic to check, check all of them if there are multiple"),
            BT::InputPort<float>("area", "area benchmark"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 
