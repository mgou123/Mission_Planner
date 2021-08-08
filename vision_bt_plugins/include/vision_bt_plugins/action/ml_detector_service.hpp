#ifndef VISION_BT_PLUGINS__ACTION__ML_DETECTOR_SERVICE_HPP_
#define VISION_BT_PLUGINS__ACTION__ML_DETECTOR_SERVICE_HPP_

#include <string>
#include <ros/ros.h>
#include <ros/serialization.h>
#include "mp_behavior_tree/bt_service_node.hpp"
#include "bbauv_msgs/MLDetector.h"

namespace mp_behavior_tree
{

    class MLDetectorService : public BtServiceNode<bbauv_msgs::MLDetector>
    {
    public:
        MLDetectorService(
            const std::string& xml_tag_name,
            const std::string& service_name,
            const BT::NodeConfiguration& conf);

        void on_tick() override;

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    BT::InputPort<std::string>("detector", "Name of detector")
                });
        }
    };

}

#endif