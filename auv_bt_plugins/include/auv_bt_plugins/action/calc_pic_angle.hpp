#ifndef AUV_BT_PLUGINS__ACTION__CALC_PIC_ANGLE_HPP_
#define AUV_BT_PLUGINS__ACTION__CALC_PIC_ANGLE_HPP_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
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
            BT::InputPort<std::string>("target", "Which pic to go"),
            BT::InputPort<float>("ratio", "[HARD CODED] ratio between camera pixel value and real-world coordinate"),
            BT::OutputPort<geometry_msgs::PoseStamped>("goal", "sideways move"),
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace mp_behavior_tree

#endif