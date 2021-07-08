// generic action for publishing data from blackboard

#ifndef MP_BEHAVIOR_TREE__BT_TOPIC_PUB_NODE_HPP_
#define MP_BEHAVIOR_TREE__BT_TOPIC_PUB_NODE_HPP_

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "mp_behavior_tree/bt_conversions.hpp"


namespace mp_behavior_tree
{
template <typename TopicT>
class BtTopicPubNode : public BT::SyncActionNode
{
public:
    BtTopicPubNode(
        const std::string & xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf)
      : BT::SyncActionNode(xml_tag_name, conf), topic_name_(topic_name), queue_size_(10)
    {
        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
        result_ = typename TopicT::ConstPtr();

        std::string remapped_topic_name;
        int queue_size;
        std::string remapped_input_key; 

        if (getInput("topic_name", remapped_topic_name)) {
            topic_name_ = remapped_topic_name;
        }

        if (getInput("queue_size", queue_size)) {
            queue_size_ = queue_size;
        }

        if (getInput("input_key", remapped_input_key)) {
            input_key_ = remapped_input_key;
        } else {
            input_key_ = topic_name_;
        }

        pub_ = std::make_shared<ros::Publisher>(node_->advertise<TopicT>(topic_name_, queue_size));

        ROS_INFO("[%s] BtTopicPubNode initialized, publishing to %s", xml_tag_name.c_str(), topic_name_.c_str());
    }

    ~BtTopicPubNode() {};

    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("topic_name", "Topic name"),
            BT::InputPort<int>("queue_size", "Topic queue size"),
            BT::InputPort<std::string>("input_key", "Name of input port to read from")
        };

        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    // mandatory to define this method
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    // Derived classes can override any of the following methods to hook into the
    // processing for the action: on_tick, on_callback, on_failure and on_success

    virtual void on_tick() {}

    virtual BT::NodeStatus on_success()
    {
        return BT::NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus on_failure()
    {
        ROS_WARN("No value received from %s", input_key_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus tick() override
    {
        on_tick();
        if (!getInput(input_key_, result_)) {
            return on_failure();
        } else {
            pub_.publish(result_);
            return on_success();
        }
    }

protected:
    std::string topic_name_;
    std::string input_key_;
    int queue_size_;

    std::shared_ptr<ros::Publisher> pub_;
    typename TopicT::ConstPtr result_;

    std::shared_ptr<ros::NodeHandle> node_;
};

} // namespace mp_behavior_tree

#endif