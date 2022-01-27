#ifndef BB_BT_PLUGINS__PLUGINS__ACTION__GO_TO_HPP_
#define BB_BT_PLUGINS__PLUGINS__ACTION__GO_TO_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "bb_msgs/LocomotionAction.h"
#include "bb_msgs/LocomotionGoal.h"
#include "bb_msgs/LocomotionResult.h"
#include "mp_behavior_tree/bt_action_node.hpp"

// mp wrapper for Locomotion action
namespace mp_behavior_tree
{

class GoTo 
: public BtActionNode<bb_msgs::LocomotionAction, 
                      bb_msgs::LocomotionGoal, 
                      bb_msgs::LocomotionResult>
{
public:
    GoTo(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination to plan to"),

            BT::InputPort<float>("forward", "Move forward (metres)"),
            BT::InputPort<float>("sideways", "Move sideways (metres)"),
            BT::InputPort<float>("depth", "Destination to plan to"),
            BT::InputPort<float>("yaw", "Yaw angle (degrees)"),
            
            BT::InputPort<double>("yaw_lock", "Yaw to lock to. If specified, relative yaw in pose will be ignored."),
            BT::InputPort<double>("depth_lock", "Depth to lock to. If specified, relative depth in pose will be ignored."),

            BT::InputPort<bool>("relative", true, "Set to true to use relative coordinates"),

            BT::InputPort<double>("sidemove_tolerance", "Tolerance for XY motion"),
            BT::InputPort<double>("forward_tolerance", "Tolerance for forward motion"),
            BT::InputPort<double>("yaw_tolerance", "Tolerance for yaw")

        });
    }
};

}


#endif