#include <string>
#include <memory>

#include "vision_bt_plugins/plugins/action/set_auv_detection_service.hpp"

namespace mp_behavior_tree
{

SetAuvDetectionService::SetAuvDetectionService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<vision::srv::DetectorAUV>(service_node_name, conf) {}

void SetAuvDetectionService::on_tick() {
    std::vector<std::string> objects;
    getInput("objects", objects);
    
}



}