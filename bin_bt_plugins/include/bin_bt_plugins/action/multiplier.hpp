#ifndef BIN_BT_PLUGINS__ACTION__MULTIPLIER_
#define BIN_BT_PLUGINS__ACTION__MULTIPLIER_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class Multiplier : public BT::SyncActionNode
{
public:
    Multiplier(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~Multiplier() {};

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<float>("orig_value", "value before multiplying"),
            BT::InputPort<float>("value_mul", "value to multiply"),
            BT::OutputPort<float>("result", "calculated result")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif