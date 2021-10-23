#ifndef BIN_BT_PLUGINS__CONDITION__IS_HANDLE_FLAT_CONDITION_
#define BIN_BT_PLUGINS__CONDITION__IS_HANDLE_FLAT_CONDITION_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "bb_msgs/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsHandleFlatCondition : public BT::ConditionNode 
{
public:
    IsHandleFlatCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsHandleFlatCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bb_msgs::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<float>("flat_ratio", "ratio between height and width of bounding box of handle that can be considered flat"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 