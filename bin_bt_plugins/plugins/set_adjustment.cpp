#include <string>
#include <memory>

#include "mp_behavior_tree/plugins/condition/choose_pic.hpp"

namespace mp_behavior_tree
{

CalcBinAngle::(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus CalcBinAngle::tick()
{   
  vision::DetectedObjects objects;
  if (!getInput("vision_objects", objects)) {
      ROS_ERROR("[SetAdjustment] objects not provided!");
      return BT::NodeStatus::FAILURE;
  } else {
      getInput("vision_objects", objects);
  }

  for (auto object : objects.detected) {
    if (object.name == "Lid") {
      count = count + 1;
    }
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::SetAdjustment>("SetAdjustment");
}