#ifndef VISION_BT_PLUGINS__ACTION__UPDATE_DETECTED_OBJECTS_HPP_
#define VISION_BT_PLUGINS__ACTION__UPDATE_DETECTED_OBJECTS_HPP_

#include <ros/ros.h>
#include "bb_msgs/DetectedObjects.h"
#include "bb_msgs/DetectedObject.h"
#include "mp_behavior_tree/bt_topic_sub_node.hpp"

namespace mp_behavior_tree
{
    class UpdateDetectedObjects : public BtTopicSubNode<bb_msgs::DetectedObjects>
    {
    public:
        UpdateDetectedObjects(
            const std::string &xml_tag_name,
            const std::string &topic_name,
            const BT::NodeConfiguration &conf);

        // mandatory to define this method
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({BT::InputPort<std::string>("prefix", "Prefix for blackboard key of each detected object")});
        }

        BT::NodeStatus on_success() override;
    };

} // namespace mp_behavior_tree

#endif