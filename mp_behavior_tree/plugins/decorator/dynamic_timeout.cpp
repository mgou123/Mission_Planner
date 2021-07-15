#include "mp_behavior_tree/plugins/decorator/dynamic_timeout.hpp"

namespace mp_behavior_tree
{
DynamicTimeout::DynamicTimeout(
    const std::string &name,
    const BT::NodeConfiguration &conf)
  : BT::DecoratorNode(name, conf),
    child_halted_(false),
    timer_id_(0),
    msec_(0),
    timeout_started_(false) {}

BT::NodeStatus DynamicTimeout::tick() {
    double sec, add_sec;

    if (!getInput("sec", sec)) {
        ROS_ERROR("[DynamicTimeout] Unable to get timeout value!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("add_sec", add_sec)) {
        add_sec = 0;
    }

    msec_ = (unsigned int)((sec + add_sec) * 1000);

    if (!timeout_started_) {
        timeout_started_ = true;
        start_ = ros::Time::now();
        setStatus(BT::NodeStatus::RUNNING);
        child_halted_ = false;

        if (msec_ > 0) {
            timer_id_ = timer_.add(std::chrono::milliseconds(msec_), 
                [this](bool aborted) 
                {
                    std::unique_lock<std::mutex> lk(timeout_mutex_);
                    if (!aborted && child()->status() == BT::NodeStatus::RUNNING) 
                    {
                        child_halted_ = true;
                        haltChild();
                    }
                });
        }
    }

    std::unique_lock<std::mutex> lk(timeout_mutex_);

    if (child_halted_) {
        timeout_started_ = false;
        setOutput("extra_sec", 0.0);
        return BT::NodeStatus::FAILURE;
    } else {
        auto child_status = child()->executeTick();
        if (child_status != BT::NodeStatus::RUNNING) {
            timeout_started_ = false;
            timeout_mutex_.unlock();
            timer_.cancel(timer_id_);
            timeout_mutex_.lock();

            setOutput("extra_sec", ((double)msec_ / 1000.0) - (ros::Time::now() - start_).toSec());
        }

        return child_status;
    }
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mp_behavior_tree::DynamicTimeout>("DynamicTimeout");
}