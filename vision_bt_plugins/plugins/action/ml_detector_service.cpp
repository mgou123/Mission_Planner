#include <string>
#include <memory>

#include "vision_bt_plugins/action/ml_detector_service.hpp"

namespace mp_behavior_tree
{

MLDetectorService::MLDetectorService(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<bbauv_msgs::MLDetector>(xml_tag_name, service_name, conf) {}

void MLDetectorService::on_tick() {
    std::string detector;

    if (!getInput("detector", detector)) {
        ROS_ERROR("[MLDetectorService] Detector needs to be specified!");
        return;
    };

    srv_->request.detector = detector;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = 
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<mp_behavior_tree::MLDetectorService>(
                name, "/auv/vision/ml/detector", config);
        };
    
    factory.registerBuilder<mp_behavior_tree::MLDetectorService>("MLDetectorService", builder);
}