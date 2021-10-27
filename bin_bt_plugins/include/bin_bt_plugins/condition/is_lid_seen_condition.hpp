#ifndef BIN_BT_PLUGINS__CONDITION__IS_LID_SEEN_CONDITION_HPP_
#define BIN_BT_PLUGINS__CONDITION__IS_LID_SEEN_CONDITION_HPP_

#include <memory>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "bb_msgs/DetectedObject.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsLidSeenCondition : public BT::ConditionNode 
{
public:
    IsLidSeenCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
        
    ~IsLidSeenCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<bb_msgs::DetectedObject>>("vision_objects", "Detected Objects"),
            BT::InputPort<int>("lid_num", "larger than or equal to lid_num of lids are seen"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 
