#ifndef MP_BEHAVIOR_TREE__ACTION__UPDATE_POSE_HPP_
#define MP_BEHAVIOR_TREE__ACTION__UPDATE_POSE_HPP_

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
};

} // namespace mp_behavior_tree

#endif