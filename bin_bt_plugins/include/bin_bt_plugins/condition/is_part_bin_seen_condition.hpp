#ifndef BIN_BT_PLUGINS__CONDITION__IS_PART_BIN_SEEN_CONDITION_HPP_
#define BIN_BT_PLUGINS__CONDITION__IS_PART_BIN_SEEN_CONDITION_HPP_

#include <memory>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree                                                                                    
{
class IsPartBinSeenCondition : public BT::ConditionNode 
{
public:
    IsPartBinSeenCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsPartBinSeenCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("vision_objects", "Detected Objects"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 