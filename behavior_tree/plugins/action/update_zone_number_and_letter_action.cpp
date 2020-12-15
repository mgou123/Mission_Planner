#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include "asv_utils/geographic_utils.h"

#include "behavior_tree/plugins/action/update_zone_number_and_letter_action.hpp"

namespace behavior_tree 
{
UpdateZoneNumberAndLetterAction::UpdateZoneNumberAndLetterAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(action_name, conf) 
{
    getInput("gps_topic", gps_topic_);

    last_zone_number_update_ = ros::Time(0);
    setOutput("last_zone_number_update", last_zone_number_update_);

    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    gps_sub_ = node_->subscribe(gps_topic_, 1, &UpdateZoneNumberAndLetterAction::gpsCallback, this);

}

BT::NodeStatus UpdateZoneNumberAndLetterAction::tick()
{
    if (last_zone_number_update_ == ros::Time(0)) {
        ROS_WARN("/clock not published yet, time is 0. Or geo pose not being published.");
        return BT::NodeStatus::RUNNING;
    }

    ROS_INFO("[UpdateZoneNumberAndLetter] Setting zone number %d in blackboard...", zone_number_);

    setOutput("zone_number", zone_number_);

    // ROS_INFO("[UpdateZoneNumberAndLetter] Setting zone letter %c in blackboard...", zone_letter_);

    setOutput("zone_letter", (uint8_t) zone_letter_);
    setOutput("last_zone_number_update", last_zone_number_update_);
    
    return BT::NodeStatus::SUCCESS;
}

void UpdateZoneNumberAndLetterAction::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

void UpdateZoneNumberAndLetterAction::gpsCallback(const sensor_msgs::NavSatFixConstPtr & msg)
{
    zone_number_ = geographic_utils::latlon_to_zone_number(msg->latitude, msg->longitude);
    zone_letter_= geographic_utils::lat_to_zone_letter(msg->latitude);
    last_zone_number_update_ = ros::Time::now();
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::UpdateZoneNumberAndLetterAction>("UpdateZoneNumberAndLetter");
}