#ifndef MP_BEHAVIOR_TREE__ACTION__UPDATE_DETECTED_OBJECTS_HPP_
#define MP_BEHAVIOR_TREE__ACTION__UPDATE_DETECTED_OBJECTS_HPP_

#include <ros/ros.h>
#include "vision/DetectedObjects.h"
#include "vision/DetectedObject.h"
#include "mp_behavior_tree/bt_topic_sub_node.hpp"

namespace mp_behavior_tree
{
class UpdateDetectedObjects : public BtTopicSubNode<vision::DetectedObjects>
{
public:
    UpdateDetectedObjects(
        const std::string & xml_tag_name,
        const std::string &topic_name,
        const BT::NodeConfiguration &conf);

    // mandatory to define this method
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("prefix", "Prefix for blackboard key of each detected object")
        });
    }

    BT::NodeStatus on_success() override;

};

} // namespace mp_behavior_tree

#endif