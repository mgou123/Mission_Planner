#include "vision_bt_plugins/action/update_detected_objects.hpp"

namespace mp_behavior_tree
{
    UpdateDetectedObjects::UpdateDetectedObjects(
        const std::string &xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf) : BtTopicSubNode<bb_msgs::DetectedObjects>(xml_tag_name, topic_name, conf)
    {

        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    }

    BT::NodeStatus UpdateDetectedObjects::on_success()
    {
        std::string prefix = "";
        std::vector<bb_msgs::DetectedObject> objects;
        for (auto object : result_->detected) {
            objects.push_back(object);
        }

        setOutput("objects", objects);
    
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<mp_behavior_tree::UpdateDetectedObjects>(
            name, "/auv3/vision/detected", config);
    };

    factory.registerBuilder<mp_behavior_tree::UpdateDetectedObjects>("UpdateDetectedObjects", builder);
}
