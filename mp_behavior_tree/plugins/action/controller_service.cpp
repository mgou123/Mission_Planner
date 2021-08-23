#include "mp_behavior_tree/plugins/action/controller_service.hpp"

namespace mp_behavior_tree
{
ControllerService::ControllerService(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<bb_msgs::Controller>(xml_tag_name, service_name, conf) {}

void ControllerService::on_tick() {
    bool flag;
    if (getInput("enable", flag)) {
      srv_->request.enable = flag;
    }
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = 
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<mp_behavior_tree::ControllerService>(
                name, "/auv/controller", config);
        };
    
    factory.registerBuilder<mp_behavior_tree::ControllerService>("ControllerService", builder);
}