#ifndef BIN_BT_PLUGINS__ACTION__ADDER_
#define BIN_BT_PLUGINS__ACTION__ADDER_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "bb_msgs/DetectedObjects.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class Adder : public BT::SyncActionNode
{
public:
    Adder(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~Adder() {};

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<float>("orig_value", "allowed offset to center on x-axis"),
            BT::InputPort<float>("value_added", "allowed offset to center on y-axis"),
            BT::OutputPort<float>("result", "calculated result")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif
