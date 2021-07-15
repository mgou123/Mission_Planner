#ifndef BIN_BT_PLUGINS__ACTION__CHOOSE_PIC_HPP_
#define BIN_BT_PLUGINS__ACTION__CHOOSE_PIC_HPP_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "vision/DetectedObjects.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{
class ChoosePic : public BT::SyncActionNode
{
public:
    ChoosePic(
       const std::string & xml_tag_name,
       const BT::NodeConfiguration & conf);
         
    ~ChoosePic() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<vision::DetectedObjects>("vision_objects", "Detected Objects"),
            BT::InputPort<std::string>("gate_side", "the side of gate chosen, Gman or Bootlegger"),
            BT::OutputPort<std::string>("pic_identifier", "the pic chosen to throw the ball"),
        };
    }
};

} // namespace mp_behavior_tree

#endif 
