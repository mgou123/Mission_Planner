#ifndef ASV_BT_PLUGINS__PLUGINS__ACTION__UPDATE_NAVSATFIX_HPP_
#define ASV_BT_PLUGINS__PLUGINS__ACTION__UPDATE_NAVSATFIX_HPP_

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "mp_behavior_tree/bt_topic_sub_node.hpp"

namespace mp_behavior_tree
{
    class UpdateNavsatfix : public BtTopicSubNode<sensor_msgs::NavSatFix>
    {
    public:
        UpdateNavsatfix(
            const std::string &xml_tag_name,
            const std::string &topic_name,
            const BT::NodeConfiguration &conf);

        // mandatory to define this method
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::OutputPort<geographic_msgs::GeoPoseStamped>("geopose", "Geopose")
            });
        }

        BT::NodeStatus on_success() override;
    };

} // namespace mp_behavior_tree

#endif