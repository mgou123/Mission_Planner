#ifndef BIN_BT_PLUGINS__CONDITION__IS_CENTER_ALIGNED_CONDITION_
#define BIN_BT_PLUGINS__CONDITION__IS_CENTER_ALIGNED_CONDITION_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "bb_msgs/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsHandleCloseEnoughCondition : public BT::ConditionNode 
{
public:
    IsHandleCloseEnoughCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsHandleCloseEnoughCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bb_msgs::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<float>("width_benchmark", "width benchmark that can be considered good to close grabber"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 