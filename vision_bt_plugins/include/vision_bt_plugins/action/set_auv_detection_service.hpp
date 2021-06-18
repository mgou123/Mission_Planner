#ifndef VISION_BT_PLUGINS__PLUGINS__ACTION__SET_DETECTION_SERVICE_HPP_
#define VISION_BT_PLUGINS__PLUGINS__ACTION__SET_DETECTION_SERVICE_HPP_

#include <string>
#include "mp_behavior_tree/bt_service_node.hpp"
#include "vision/srv/DetectorAUV.h"

namespace mp_behavior_tree
{

class SetAuvDetectionService : public BtServiceNode<vision::srv::DetectorAUV>
{
public:
    SetAuvDetectionService(
        const std::string & service_node_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<std::vector<std::string>>("objects", "names of objects to be detected")
        });
    }
};

}

#endif