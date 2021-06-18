#ifndef AUV_BT_PLUGINS__PLUGINS__ACTION__LOCOMOTION_ACTION_HPP_
#define AUV_BT_PLUGINS__PLUGINS__ACTION__LOCOMOTION_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_utils/action/locomotion.hpp"
#include "mp_behavior_tree/bt_action_node.hpp"

// mp wrapper for Locomotion action
namespace mp_behavior_tree
{

class LocomotionAction : public BtActionNode<nav_utils::action::Locomotion>
{
public:
    LocomotionAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        });
    }
};

} // namespace mp_behavior_tree


#endif