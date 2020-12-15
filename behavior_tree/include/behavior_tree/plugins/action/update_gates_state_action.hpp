// updates the state of the asv in gymkhana gates task based on gate positions

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_GATES_STATE_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_GATES_STATE_ACTION_HPP_

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "asv_msgs/DetectedObject.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{
class UpdateGatesStateAction : public BT::SyncActionNode
{
public:
    UpdateGatesStateAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    UpdateGatesStateAction() = delete;
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<asv_msgs::DetectedObject>>("detected_objects", "Objects detected by perception"),
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose"),
            BT::OutputPort<std::string>("gates_state", "State of gates task for asv to choose move action"),
            BT::OutputPort<geometry_msgs::PoseStamped>("gate_1", "Closest gate to asv"),
            BT::OutputPort<geometry_msgs::PoseStamped>("gate_2", "Second-closest gate to asv")
        };
    }

private:
    std::vector<geometry_msgs::PoseStamped> cylinder_objects_;
    geometry_msgs::Pose robot_pose_;

    std::string gates_state_;
    geometry_msgs::PoseStamped gate_1_;
    geometry_msgs::PoseStamped gate_2_;
};

} // namespace behavior_tree

#endif