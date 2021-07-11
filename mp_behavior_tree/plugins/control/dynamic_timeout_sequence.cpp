#include "mp_behavior_tree/plugins/control/dynamic_timeout_sequence.hpp"

namespace mp_behavior_tree
{
DynamicTimeoutSequence::DynamicTimeoutSequence(
    const std::string &name,
    const BT::NodeConfiguration &conf)
  : BT::ControlNode(name, conf),
    current_child_idx_(0) {}

void DynamicTimeoutSequence::halt() {
    current_child_idx_ = 0;
    ControlNode::halt();
}

BT::NodeStatus DynamicTimeoutSequence::tick() 
{
    

    const size_t children_count = children_nodes_.size();
    const size_t timeout_children_count = 0;
    for (auto child : children_nodes_) {
        if child.registrationID

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

            case BT::NodeStatus::FAILURE:
            {
                

            }
        }


    }


}



}