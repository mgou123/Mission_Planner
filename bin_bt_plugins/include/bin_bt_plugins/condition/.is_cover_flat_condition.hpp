#ifndef BIN_BT_PLUGINS__CONDITION__IS_COVER_FLAT_CONDITION_
#define BIN_BT_PLUGINS__CONDITION__IS_COVER_FLAT_CONDITION_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "bb_msgs/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsCoverFlatCondition : public BT::ConditionNode 
{
public:
    IsCoverFlatCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsCoverFlatCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bb_msgs::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<float>("angle_range", "accepted angle range that the cover is considered flat"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 