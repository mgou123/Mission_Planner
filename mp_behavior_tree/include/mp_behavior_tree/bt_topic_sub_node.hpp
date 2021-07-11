// generic action for writing data to blackboard

#ifndef MP_BEHAVIOR_TREE__BT_TOPIC_SUB_NODE_HPP_
#define MP_BEHAVIOR_TREE__BT_TOPIC_SUB_NODE_HPP_

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "mp_behavior_tree/bt_conversions.hpp"


namespace mp_behavior_tree
{
template <typename TopicT>
class BtTopicSubNode : public BT::SyncActionNode
{
public:
    BtTopicSubNode(
        const std::string & xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf)
      : BT::SyncActionNode(xml_tag_name, conf), topic_name_(topic_name), queue_size_(10)
    {
        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
        result_ = typename TopicT::ConstPtr();
        ROS_INFO("[%s] BtTopicSubNode initialized", xml_tag_name.c_str());
    }

    ~BtTopicSubNode() {};

    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("topic_name", "Topic name"),
            BT::InputPort<int>("queue_size", "Topic queue size"),
            BT::InputPort<double>("timeout_sec", "Timeout for waiting for message publish"),
            BT::OutputPort<TopicT>("result", "Output port to write message result to"),
        };

        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    // mandatory to define this method
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    void subscribe() {
        std::string remapped_topic_name;
        int queue_size;
        std::string remapped_output_key;
        double timeout_sec;

        if (getInput("topic_name", remapped_topic_name)) {
            topic_name_ = remapped_topic_name;
        }

        if (getInput("queue_size", queue_size)) {
            queue_size_ = queue_size;
        }

        if (getInput("output_key", remapped_output_key)) {
            output_key_ = remapped_output_key;
        } else {
            output_key_ = topic_name_;
        }

        if (getInput("timeout_sec", timeout_sec)) {
            timeout_sec_ = timeout_sec;
        } else {
            timeout_sec_ = 2.0;
        }

        sub_ = std::make_shared<ros::Subscriber>(node_->subscribe(topic_name_, queue_size_, &BtTopicSubNode::callback, this));
        ROS_INFO("[%s] BtTopicSubNode subscribed to %s", name().c_str(), topic_name_.c_str());
        subscribed_ = true;
    }

    // Derived classes can override any of the following methods to hook into the
    // processing for the action: on_tick, on_callback, on_failure and on_success

    virtual void on_tick() {}

    virtual void on_callback() {}

    virtual BT::NodeStatus on_success()
    {
        return BT::NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus on_failure()
    {
        //ROS_WARN("No messages received over %s", topic_name_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus tick() override
    {
        on_tick();

        if (!subscribed_) {
            subscribe();

            // wait for first message to come in
            auto start = ros::Time::now();
            while (first_time_ && (ros::Time::now() - start).toSec() < timeout_sec_);
        }

        if (first_time_) {
            ROS_ERROR("[TopicSubNode] Unable to receive messages from topic %s", topic_name_.c_str());
            return on_failure();
        } else {
            setOutput("result", *result_);
            return on_success();
        }
    }

protected:
    void callback(const typename TopicT::ConstPtr & msg)
    {
        result_ = msg;
        if (first_time_) {
            first_time_ = false;
        }

        on_callback();
    }

    std::string topic_name_;
    std::string output_key_;
    int queue_size_;
    double timeout_sec_;

    bool subscribed_{false};
    bool first_time_{true};

    std::shared_ptr<ros::Subscriber> sub_;
    typename TopicT::ConstPtr result_;

    std::shared_ptr<ros::NodeHandle> node_;
};

} // namespace mp_behavior_tree

#endif