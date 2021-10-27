#ifndef BIN_BT_PLUGINS__ACTION__CALC_PIC_ANGLE_
#define BIN_BT_PLUGINS__ACTION__CALC_PIC_ANGLE_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "bb_msgs/DetectedObject.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class CalcPicAngle : public BT::SyncActionNode
{
public:
    CalcPicAngle(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~CalcPicAngle() {};

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<bb_msgs::DetectedObject>>("vision_objects", "Detected Objects"),
            BT::InputPort<std::string>("pic_identifier", "Which pic to go"),
            // BT::InputPort<float>("center_offset_x", "allowed offset to center on x-axis"),
            // BT::InputPort<float>("center_offset_y", "allowed offset to center on y-axis"),
            BT::InputPort<float>("ratio", "ratio between camera pixel value and real-world coordinate"),
            BT::OutputPort<float>("x_goal", "sideways move"),
            BT::OutputPort<float>("y_goal", "forward move"),
            BT::OutputPort<float>("yaw_goal", "yaw move")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif 
