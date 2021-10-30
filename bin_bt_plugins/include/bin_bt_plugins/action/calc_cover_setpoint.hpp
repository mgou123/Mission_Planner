#ifndef BIN_BT_PLUGINS__ACTION__CALC_COVER_SETPOINT_
#define BIN_BT_PLUGINS__ACTION__CALC_COVER_SETPOINT_

#include <memory>
#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "ros/ros.h"
#include "bb_msgs/DetectedObject.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class CalcCoverSetpoint : public BT::SyncActionNode
{
public:
    CalcCoverSetpoint(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~CalcCoverSetpoint() {};

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<bb_msgs::DetectedObject>>("vision_objects", "Detected Objects"),
            BT::InputPort<float>("ratio", "ration between real world distance and number of pixels"),
            BT::InputPort<float>("lift_ratio_x", "relative x ratio of picking point"),
            BT::InputPort<float>("lift_ratio_y", "relatvie y ratio of picking point"),
            BT::InputPort<float>("pre_direction", "previous direction to throw the cover if this is the second cover, 0 if this is the first cover"),
            BT::OutputPort<float>("lift_pose_1_x", "relative sideways position to lift first cover"),
            BT::OutputPort<float>("lift_pose_1_y", "relative forward position to lift first cover"),
            BT::OutputPort<float>("drop_pose_1", "relative forward position to drop first cover"),
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif 