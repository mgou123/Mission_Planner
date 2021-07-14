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
            BT::InputPort<double>("extra_sec", "Time left over at current child node"),
            BT::OutputPort<double>("add_sec", "Additional time awarded to next dynamic timeout child node")
        };
    }

    void halt() override;

private:
    BT::NodeStatus tick() override;
    bool is_child_timeout_node(TreeNode* child);
    double divide_extra_time();

    size_t current_child_idx_;
    int timeout_children_count_;
    double bank_;
    std::string rule_;

};

} // namespace mp_behavior_tree



#endif