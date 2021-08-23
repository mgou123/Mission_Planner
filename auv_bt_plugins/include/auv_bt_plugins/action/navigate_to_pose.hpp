#ifndef AUV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_TO_POSE_HPP_
#define AUV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_TO_POSE_HPP_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "bb_msgs/LocomotionAction.h"
#include "bb_msgs/LocomotionGoal.h"
#include "bb_msgs/LocomotionResult.h"
#include "mp_behavior_tree/bt_action_node.hpp"

// mp wrapper for Locomotion action
namespace mp_behavior_tree
{

class NavigateToPose 
: public BtActionNode<bb_msgs::LocomotionAction, 
                      bb_msgs::LocomotionGoal, 
                      bb_msgs::LocomotionResult>
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
            BT::InputPort<double>("yaw_lock", "Yaw to lock to. If specified, relative yaw in pose will be ignored."),
            BT::InputPort<double>("yaw_lock_relative", "Behavior of yaw_lock. If true, yaw_lock will be treated as relative."),
            BT::InputPort<double>("depth_lock", "Depth to lock to. If specified, relative depth in pose will be ignored."),
            BT::InputPort<bool>("relative", "Set to true to use relative coordinates"),
            BT::InputPort<double>("sidemove_tolerance", "Tolerance for XY motion"),
            BT::InputPort<double>("forward_tolerance", "Tolerance for forward motion"),
            BT::InputPort<double>("yaw_tolerance", "Tolerance for yaw")

        });
    }
};

} // namespace mp_behavior_tree


#endif