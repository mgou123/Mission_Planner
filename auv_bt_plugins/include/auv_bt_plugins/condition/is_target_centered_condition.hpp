#ifndef AUV_BT_PLUGINS__CONDITION__IS_TARGET_CENTERED_CONDITION_HPP_
#define AUV_BT_PLUGINS__CONDITION__IS_TARGET_CENTERED_CONDITION_HPP_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsTargetCenteredCondition : public BT::ConditionNode
{
public:
    IsTargetCenteredCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    ~IsTargetCenteredCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<std::string>("pic_identifier", "Which pic to go"),
            BT::InputPort<float>("center_offset_x", "allowed offset to center on x-axis"),
            BT::InputPort<float>("center_offset_y", "allowed offset to center on y-axis"),
            BT::InputPort<float>("x_ratio", "target relative ratio of x"),
            BT::InputPort<float>("y_ratio", "target relative ratio of y"),
        };
    }
};

} // namespace mp_behavior_tree

#endif
