#include "asv_bt_plugins/action/update_geopose.hpp"

namespace mp_behavior_tree
{
    UpdateGeopose::UpdateGeopose(
        const std::string &xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf) : BtTopicSubNode<geographic_msgs::GeoPoseStamped>(xml_tag_name, topic_name, conf)
    {

        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    }

    BT::NodeStatus UpdateGeopose::on_success()
    {
        geographic_msgs::GeoPoseStamped geopose;
        setOutput("geopose", *result_);
    
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<mp_behavior_tree::UpdateGeopose>(
            name, "/auv3/vision/detected", config);
    };

    factory.registerBuilder<mp_behavior_tree::UpdateGeopose>("UpdateGeopose", builder);
}
