#ifndef VISION_BT_PLUGINS__ACTION__SET_DETECTION_SERVICE_HPP_
#define VISION_BT_PLUGINS__ACTION__SET_DETECTION_SERVICE_HPP_

#include <string>
#include <ros/ros.h>
#include <ros/serialization.h>
#include "mp_behavior_tree/bt_service_node.hpp"
#include "vision/DetectorAUV.h"

namespace mp_behavior_tree
{

    class DetectorAUVService : public BtServiceNode<vision::DetectorAUV>
    {
    public:
        DetectorAUVService(
            const std::string& xml_tag_name,
            const std::string& service_name,
            const BT::NodeConfiguration& conf);

        void on_tick() override;

        BT::NodeStatus on_success() override;

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    BT::InputPort<std::vector<std::string>>("objects", "names of objects to be detected"),
                    BT::InputPort<bool>("enable", "True to enable vision. False to disable. Defaults to True")
                });
        }
    };

}

#endif