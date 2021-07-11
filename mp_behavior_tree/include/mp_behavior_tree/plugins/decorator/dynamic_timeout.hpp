#ifndef MP_BEHAVIOR_TREE__PLUGINS__DECORATOR__DYNAMIC_TIMEOUT_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__DECORATOR__DYNAMIC_TIMEOUT_HPP_

#include <atomic>

#include <ros/ros.h>

#include <behaviortree_cpp_v3/decorators/timer_queue.h>
#include <behaviortree_cpp_v3/decorator_node.h>

namespace mp_behavior_tree
{

// similar to behaviortreeCPP timeout node with added dynamic functionality
// to be used with dynamic timeout sequence node, or individually
class DynamicTimeout : public BT::DecoratorNode
{
public: 
    DynamicTimeout(
        const std::string &name,
        const BT::NodeConfiguration &conf);
    
    ~DynamicTimeout() {
        timer_.cancelAll();
    }
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("sec", "After a certain amount of time, halt() the child if it's still running"),
            BT::InputPort<double>("add_sec", "Additional time carried over from previous nodes"),
            BT::OutputPort<double>("extra_sec", "Time left over at this node")
        };
    }

private:
    BT::NodeStatus tick() override;

    BT::TimerQueue<> timer_;
    
    std::atomic<bool> child_halted_;
    uint64_t timer_id_;

    unsigned int msec_;
    bool timeout_started_;
    ros::Time start_;

    std::mutex timeout_mutex_;

};

} // namespace mp_behavior_tree

#endif
