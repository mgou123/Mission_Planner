// generic action for writing data to blackboard

#ifndef MP_BEHAVIOR_TREE__BT_TOPIC_SUB_NODE_HPP_
#define MP_BEHAVIOR_TREE__BT_TOPIC_SUB_NODE_HPP_

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

        std::string remapped_topic_name;
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

        sub_ = std::make_shared<node_->subscribe>(topic_name_, queue_size_, &BtTopicNode::callback, this);

        ROS_INFO("\"%s\" BtTopicSubNode initialized", xml_tag_name.c_str(), topic_name_.c_str());
    }

    BtTopicSubNode() = delete;

    virtual ~BtTopicSubNode() = delete;

    static BT::PortsList providedBasicPorts(BT::PortList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("topic_name", "Topic name"),
            BT::InputPort<int>("queue_size", "Topic queue size"),
            BT::InputPort<std::string>("output_key", "Name of output port to write to");
            BT::OutputPort<TopicT>(output_key_, "Output port");
        };

        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    // mandatory to define this method
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(());
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
        ROS_WARN("No messages received over %s", topic_name_);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus tick() override
    {
        on_tick();
        if (first_time) {
            return on_failure();
        } else {
            setOutput(output_key_, *result_);
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

    bool first_time_{true};

    ros::Subscriber sub_;
    typename TopicT::ConstPtr result_;

    std::shared_ptr<ros::NodeHandle> node_;
};

} // namespace mp_behavior_tree

#endif