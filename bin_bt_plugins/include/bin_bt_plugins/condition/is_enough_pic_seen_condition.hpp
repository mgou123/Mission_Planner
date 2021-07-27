#ifndef BIN_BT_PLUGINS__CONDITION__IS_ENOUGH_PIC_SEEN_CONDITION_HPP_
#define BIN_BT_PLUGINS__CONDITION__IS_ENOUGH_PIC_SEEN_CONDITION_HPP_

#include <memory>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsEnoughPicSeenCondition : public BT::ConditionNode 
{
public:
    IsEnoughPicSeenCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsEnoughPicSeenCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<int>("pic_num", "larger than or equal to pic_num of pics are seen"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 
