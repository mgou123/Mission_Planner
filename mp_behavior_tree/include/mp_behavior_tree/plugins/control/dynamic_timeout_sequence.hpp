#ifndef MP_BEHAVIOR_TREE__PLUGINS__CONTROL__DYNAMIC_TIMEOUT_SEQUENCE_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__CONTROL__DYNAMIC_TIMEOUT_SEQUENCE_HPP_

// to be used with dynamic timeout nodes to dynamically allocate timeout
// otherwise, acts like a normal sequence node

#include <behaviortree_cpp_v3/control_node.h>

namespace mp_behavior_tree
{

class DynamicTimeoutSequence : public BT::ControlNode
{
public:
    DynamicTimeoutSequence(
        const std::string &name,
        const BT::NodeConfiguration &conf);
    
    ~DynamicTimeoutSequence() {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("rule", "Rule for dynamic division of timer: ALL or EQUAL"),
            BT::InputPort<std::string>("dynamic_timeout_key", "Port key for setting dynamic timeout of current child"),
            BT::InputPort<double>("elapsed", "Seconds elapsed since current child was ticked")
        };
    }

    void halt() override;

private:
    BT::NodeStatus tick() override;

    size_t current_child_idx_;
    double bank_;

};

} // namespace mp_behavior_tree



#endif