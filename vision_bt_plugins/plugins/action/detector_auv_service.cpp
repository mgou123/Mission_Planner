#include <string>
#include <memory>

#include "vision_bt_plugins/action/detector_auv_service.hpp"

namespace mp_behavior_tree
{

DetectorAUVService::DetectorAUVService(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<vision::DetectorAUV>(xml_tag_name, service_name, conf) {}

void DetectorAUVService::on_tick() {
    std::vector<std::string> objects;

    getInput("objects", objects);
    if (!objects.size()) {
      ROS_WARN("No objects provided in input port! Setting all object fields to false..");
      return;
    }
    yaml_req_ = "{";
    for (auto object : objects) {
      yaml_req_ += "'" + object + "':true, ";  
    }
    yaml_req_ += "}";
}

BT::NodeStatus DetectorAUVService::on_success() {
  if(!yaml_req_.empty()) {
    ROS_INFO("Service call completed. Response: %s", yaml_res_.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = 
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<mp_behavior_tree::DetectorAUVService>(
                name, "/auv/vision/detector", config);
        };
    
    factory.registerBuilder<mp_behavior_tree::DetectorAUVService>("DetectorAUVService", builder);
}