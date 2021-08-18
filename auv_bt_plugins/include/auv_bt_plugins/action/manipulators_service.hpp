#ifndef AUV_BT_PLUGINS__ACTION__MANIPULATORS_SERVICE_HPP_
#define AUV_BT_PLUGINS__ACTION__MANIPULATORS_SERVICE_HPP_

#include <string>
#include <ros/ros.h>
#include "mp_behavior_tree/bt_service_node.hpp"
#include "bb_msgs/Manipulators.h"

namespace mp_behavior_tree
{

class ManipulatorsService : public BtServiceNode<bb_msgs::Manipulators>
{
public:
    ManipulatorsService(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<bool>("top_torpedo", false, "True to fire top torpedo, false to disable it"),
            BT::InputPort<bool>("bottom_torpedo", false, "True to fire bottom torpedo, false to disable it"),
            BT::InputPort<bool>("dropper", false, "True to drop, false to disable it"),
            BT::InputPort<bool>("grabber", false, "True to grab, false to disable it"),
            BT::InputPort<bool>("extend_linear", false, "True to extend linear, false to disable it"),
            BT::InputPort<bool>("retract_linear", false, "True to retract linear, false to disable it")
        });
    }
};

}

#endif