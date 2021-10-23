#include <string>
#include <memory>

#include "bin_bt_plugins/action/choose_pic.hpp"

namespace mp_behavior_tree
{

ChoosePic::ChoosePic(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus ChoosePic::tick()
{   
  bb_msgs::DetectedObjects objects; 
  std::string gate_side;
  int task_identifier;
  int count_gman;
  int count_bootlegger; 
  int area_one = 0;
  int area_two = 0; 
  std::string pic_chosen;
  int pic_num = 0; 

  if (!getInput("vision_objects", objects)) {
    ROS_ERROR("[ChoosePic] objects not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("vision_objects", objects);
  }
  
  if (!getInput("gate_side", gate_side)) {
    ROS_ERROR("[ChoosePic] gate_side not provided!");
    return BT::NodeStatus::FAILURE;
  } else {
    getInput("gate_side", gate_side);
  }

  if (gate_side == "Gman") {
    task_identifier = 1; 
  } else if (gate_side == "Bootlegger") {
    task_identifier = 0; 
  } else {
    ROS_ERROR("[ChoosePic] Invalid side of gate!");
  }

  for (auto object : objects.detected) {
    pic_num += 1; 
    if (task_identifier == 0) {
      if (object.name == "Barrel") {
        count_bootlegger += 1;
        area_one = object.bbox_height * object.bbox_width;
      } else if (object.name == "Wiskey Bottle") {
        count_bootlegger += 1; 
        area_two = object.bbox_height * object.bbox_width;
      }
    } else {
      if (object.name == "Notepad") {
        count_gman += 1; 
        area_one = object.bbox_height * object.bbox_width;
      } else if (object.name == "Telephone") {
        count_gman += 1; 
        area_two = object.bbox_height * object.bbox_width;
      }
    }
  }

  if (task_identifier == 0) {
    if (count_bootlegger == 0) {
      ROS_WARN("[ChoosePic] no pic in sight");
      return BT::NodeStatus::FAILURE;
    } else if (count_bootlegger == 1) {
      if (area_one != 0) {
        setOutput("pic_identifier", "Barrel");
        pic_chosen = "Barrel";
        return BT::NodeStatus::SUCCESS;
      } else {
        setOutput("pic_identifier", "Wiskey Bottle");
        pic_chosen = "Wiskey Bottle";
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      if (area_one > area_two) {
        setOutput("pic_identifier", "Barrel");
        pic_chosen = "Barrel";
        return BT::NodeStatus::SUCCESS;
      } else {
        setOutput("pic_identifier", "Wiskey Bottle");
        pic_chosen = "Wiskey Bottle";
        return BT::NodeStatus::SUCCESS;
      }
    }
  } else {
    if (count_gman == 0) {
      ROS_WARN("[ChoosePic] no pic in sight");
      return BT::NodeStatus::FAILURE;
    } else if (count_gman == 1) {
      if ( area_one != 0) {
        setOutput("pic_identifier", "Notepad");
        pic_chosen = "Notepad";
        return BT::NodeStatus::SUCCESS;
      } else {
        setOutput("pic_identifier", "Telephone");
        pic_chosen = "Telephone";
        return BT::NodeStatus::SUCCESS;
      }
    } 
    if (area_one > area_two) {
      setOutput("pic_identifier", "Notepad");
      pic_chosen = "Notepad";
      return BT::NodeStatus::SUCCESS;
    } else {
      setOutput("pic_identifier", "Telephone");
      pic_chosen = "Telephone";
      return BT::NodeStatus::SUCCESS;
    }
  } 

  ROS_INFO("choose pic running, pic chosen is %s", pic_chosen);
  
  return BT::NodeStatus::FAILURE;
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::ChoosePic>("ChoosePic");
}