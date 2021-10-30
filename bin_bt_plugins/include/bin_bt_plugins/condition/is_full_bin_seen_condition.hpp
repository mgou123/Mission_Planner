#ifndef BIN_BT_PLUGINS__CONDITION__IS_FULL_BIN_SEEN_CONDITION_HPP_
#define BIN_BT_PLUGINS__CONDITION__IS_FULL_BIN_SEEN_CONDITION_HPP_

#include <memory>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "bb_msgs/DetectedObject.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsFullBinSeenCondition : public BT::ConditionNode 
{
public:
    IsFullBinSeenCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsFullBinSeenCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<bb_msgs::DetectedObject>>("vision_objects", "Detected Objects"),
            BT::InputPort<float>("area_benchmark", "area benchmark that can be considered fully seen bin"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 
