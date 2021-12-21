#include "asv_bt_plugins/action/update_navsatfix.hpp"

namespace mp_behavior_tree
{
    UpdateNavsatfix::UpdateNavsatfix(
        const std::string &xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf) : BtTopicSubNode<sensor_msgs::NavSatFix>(xml_tag_name, topic_name, conf)
    {

        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    }

    BT::NodeStatus UpdateNavsatfix::on_success()
    {
        geographic_msgs::GeoPoseStamped geopose;
        geopose.pose.position.latitude = result_->latitude;
        geopose.pose.position.longitude = result_->longitude;
        geopose.pose.position.altitude = result_->altitude;
        setOutput("geopose", geopose);
    
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<mp_behavior_tree::UpdateNavsatfix>(
            name, "/auv3/vision/detected", config);
    };

    factory.registerBuilder<mp_behavior_tree::UpdateNavsatfix>("UpdateNavsatfix", builder);
}
