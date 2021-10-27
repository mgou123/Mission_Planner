#ifndef MP_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_POSE_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_POSE_HPP_

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "mp_behavior_tree/bt_topic_sub_node.hpp"

namespace mp_behavior_tree
{
class UpdatePose : public BtTopicSubNode<nav_msgs::Odometry>
{
public:
    UpdatePose(
        const std::string & xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf);
    
    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::OutputPort<nav_msgs::Odometry>("result", "Odometry output"),
            BT::OutputPort<double>("depth", "Port to write depth value to") // hack job for now to correct for depth
        });
    }

    BT::NodeStatus on_success() override;
};

} // namespace mp_behavior_tree

#endif