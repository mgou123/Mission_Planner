#include "mp_behavior_tree/plugins/action/update_pose.hpp"

namespace mp_behavior_tree
{
UpdatePose::UpdatePose(
        const std::string & xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf) : BtTopicSubNode<nav_msgs::Odometry>(xml_tag_name, topic_name, conf) {}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = 
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<mp_behavior_tree::UpdatePose>(
                name, "/auv/nav/odom_ned", config);
        };
    
    factory.registerBuilder<mp_behavior_tree::UpdatePose>("UpdatePose", builder);
}