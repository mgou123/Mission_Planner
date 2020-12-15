// Updates current zone number of the asv

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_ZONE_NUMBER_AND_LETTER_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_ZONE_NUMBER_AND_LETTER_ACTION_HPP_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "sensor_msgs/NavSatFix.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class UpdateZoneNumberAndLetterAction : public BT::ActionNodeBase
{
public:
    UpdateZoneNumberAndLetterAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    UpdateZoneNumberAndLetterAction() = delete;

    BT::NodeStatus tick() override;
    
    void halt() override;
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("gps_topic", "Topic for gps"),
            BT::OutputPort<int>("zone_number", "UTM zone number of the current robot pose"),
            BT::OutputPort<uint8_t>("zone_letter", "UTM zone letter of the current robot pose"),
            BT::OutputPort<ros::Time>("last_zone_number_update", "timestamp for latest zone number update")
        };
    }

private:
    void gpsCallback(const sensor_msgs::NavSatFixConstPtr & msg);
    
    std::shared_ptr<ros::NodeHandle> node_;
    ros::Subscriber gps_sub_;
    std::string gps_topic_;
    int zone_number_;
    char zone_letter_;

    ros::Time last_zone_number_update_;
};

} // namespace behavior_tree

#endif 