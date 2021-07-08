#ifndef AUV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_TO_POSE_HPP_
#define AUV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_TO_POSE_HPP_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "nav_utils/LocomotionAction.h"
#include "nav_utils/LocomotionGoal.h"
#include "nav_utils/LocomotionResult.h"
#include "mp_behavior_tree/bt_action_node.hpp"

// mp wrapper for Locomotion action
namespace mp_behavior_tree
{

class NavigateToPose 
: public BtActionNode<nav_utils::LocomotionAction, 
                      nav_utils::LocomotionGoal, 
                      nav_utils::LocomotionResult>
{
public:
    NavigateToPose(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination to plan to"),
            BT::InputPort<bool>("relative", "Set to true to use relative coordinates")
        });
    }
};

} // namespace mp_behavior_tree


#endif