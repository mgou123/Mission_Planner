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
  vision::DetectedObject pic; 
  std::string identifier;

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[CalcBinAngle] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }

  if (!getInput("pic_identifier", identifier)) {
    ROS_ERROR("[CalcBinAngle] pic_identifier not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("pic_identifier", identifier);
  }

  for (auto object : objects.detected) {
    if (object.name == identifier) {
      pic = object;
    }
  }

  setOutput("angle", angle);
  return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::CalcBinAngle>("CalcBinAngle");
}