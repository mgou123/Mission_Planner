#ifndef BB_BT_PLUGINS__PLUGINS__ACTION__CONTROLLER_SERVICE_HPP_
#define BB_BT_PLUGINS__PLUGINS__ACTION__CONTROLLER_SERVICE_HPP_

#include <ros/ros.h>
#include "mp_behavior_tree/bt_service_node.hpp"
#include "bb_msgs/Controller.h"

namespace mp_behavior_tree
{
class ControllerService : public BtServiceNode<bb_msgs::Controller>
{
public:
    ControllerService(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<bool>("enable", "True to enable control, false to disable control")
        });
    }
};

} // namespace mp_behavior_tree

#endif