#include "mp_behavior_tree/plugins/control/dynamic_timeout_sequence.hpp"

namespace mp_behavior_tree
{
DynamicTimeoutSequence::DynamicTimeoutSequence(
    const std::string &name,
    const BT::NodeConfiguration &conf)
  : BT::ControlNode(name, conf),
    current_child_idx_(0),
    timeout_children_count_(0) {}

void DynamicTimeoutSequence::halt() {
    current_child_idx_ = 0;
    ControlNode::halt();
}

BT::NodeStatus DynamicTimeoutSequence::tick() 
{

    const size_t children_count = children_nodes_.size();

    timeout_children_count_ = 0;

    for (int i = current_child_idx_ + 1; i < children_count; i++) {
        if (is_child_timeout_node(children_nodes_[i])) {
            timeout_children_count_++;
        }
    }

    std::string rule;

    if (getInput("rule", rule)) {
        rule_ = rule;
    }

    setStatus(BT::NodeStatus::RUNNING);
        
    while(current_child_idx_ < children_count) {
        TreeNode* current_child_node = children_nodes_[current_child_idx_];
        const BT::NodeStatus child_status = current_child_node->executeTick();

        switch (child_status) {
            case BT::NodeStatus::RUNNING:
            {
                return child_status;
            }

            case BT::NodeStatus::FAILURE: {} // failure and success have similar actions

            case BT::NodeStatus::SUCCESS:
            {
                if (is_child_timeout_node(current_child_node)) {
                    double extra_sec;
                    if (getInput("extra_sec", extra_sec)) {
                        bank_ += extra_sec;
                    }
                }

                setOutput("add_sec", divide_extra_time());
                current_child_idx_++;
            }
            break;

            case BT::NodeStatus::IDLE:
            {
                throw BT::LogicError("A child node must never return IDLE");
            }
        } // end switch
    }     // end while loop

    // The entire while loop completed.
    if (current_child_idx_ == children_count) {
        haltChildren();
        current_child_idx_ = 0;
    }

    return BT::NodeStatus::SUCCESS;
}


bool DynamicTimeoutSequence::is_child_timeout_node(TreeNode* child) {
    return child->registrationName() == "DynamicTimeout";
}

double DynamicTimeoutSequence::divide_extra_time() {
    if (rule_ == "EQUAL") {
        double add = bank_ / (double)timeout_children_count_;
        bank_ -= add;
        return add;
    } else if (rule_ == "ALL") {
        double add = bank_;
        bank_ = 0;
        return add;
    } else {
        double add = bank_ / (double)timeout_children_count_;
        bank_ -= add;
        return add;
    }
}

} // namespace mp_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mp_behavior_tree::DynamicTimeoutSequence>("DynamicTimeoutSequence");
}