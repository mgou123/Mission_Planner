#ifndef AUV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_CONTROL_HPP_
#define AUV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_CONTROL_HPP_

#include <string>

#include "nav_utils/LocomotionAction.h"
#include "nav_utils/LocomotionGoal.h"
#include "nav_utils/LocomotionResult.h"
#include "mp_behavior_tree/bt_action_node.hpp"

// mp wrapper for Locomotion action
namespace mp_behavior_tree
{

class NavigateControl 
: public BtActionNode<nav_utils::LocomotionAction, 
                      nav_utils::LocomotionGoal, 
                      nav_utils::LocomotionResult>
{
public:
    NavigateControl(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;
    void on_wait_for_result() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<float>("forward", "Destination to plan to"),
            BT::InputPort<float>("sideways", "Destination to plan to"),
            BT::InputPort<float>("depth", "Destination to plan to"),
            BT::InputPort<float>("yaw", "Destination to plan to"),
        });
    }
};

} // namespace mp_behavior_tree


#endif