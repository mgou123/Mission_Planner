#ifndef AUV_BT_PLUGINS__ACTION__MANIPULATORS_SERVICE_HPP_
#define AUV_BT_PLUGINS__ACTION__MANIPULATORS_SERVICE_HPP_

#include <string>
#include <ros/ros.h>
#include "mp_behavior_tree/bt_service_node.hpp"
#include "bbauv_msgs/Manipulators.h"

namespace mp_behavior_tree
{

class ManipulatorsService : public BtServiceNode<bbauv_msgs::Manipulators>
{
public:
    ManipulatorsService(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {});
    }
};

}

#endif