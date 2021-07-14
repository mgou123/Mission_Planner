#ifndef BIN_BT_PLUGINS__CALC_PIC_ANGLE_
#define BIN_BT_PLUGINS__CALC_PIC_ANGLE_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class CalcBinAngle : public BT::SyncActionNode
{
public:
    CalcBinAngle::CalcBinAngle(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~CalcBinAngle() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("detected_objects", "Detected Objects"),
            BT::InputPort<std::string>("pic_identifier", "the pic chosen to throw the ball"),
            BT::OutputPort<float>("angle", "angle to turn to face the object"),
        };
    }
}

} // namespace mp_behavior_tree

#endif 
