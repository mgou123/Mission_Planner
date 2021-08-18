#include <string>
#include <memory>

#include "auv_bt_plugins/action/manipulators_service.hpp"

namespace mp_behavior_tree
{

ManipulatorsService::ManipulatorsService(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<bb_msgs::Manipulators>(xml_tag_name, service_name, conf) {}

void ManipulatorsService::on_tick() {
    getInput("top_torpedo", srv_->request.top_torpedo);
    getInput("bottom_torpedo", srv_->request.bottom_torpedo);
    getInput("dropper", srv_->request.dropper);
    getInput("grabber", srv_->request.grabber);
    getInput("extend_linear", srv_->request.extend_linear);
    getInput("retract_linear", srv_->request.retract_linear);
    
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = 
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<mp_behavior_tree::ManipulatorsService>(
                name, "/manipulators", config);
        };
    
    factory.registerBuilder<mp_behavior_tree::ManipulatorsService>("ManipulatorsService", builder);
}