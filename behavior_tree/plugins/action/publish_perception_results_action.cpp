#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include <geographic_msgs/GeoPoseStamped.h>
#include "asv_msgs/DetectedObject.h"
#include "asv_utils/geographic_utils.h"

#include "behavior_tree/plugins/action/publish_perception_results_action.hpp"

namespace behavior_tree
{
PublishPerceptionResultsAction::PublishPerceptionResultsAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{
    getInput("perception_task_topic", perception_task_topic_);
    
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    perception_task_pub_ = node_->advertise<geographic_msgs::GeoPoseStamped>(perception_task_topic_, 1000);
}

BT::NodeStatus PublishPerceptionResultsAction::tick() 
{
    std::vector<asv_msgs::DetectedObject> detected_objects;
    int zone_number;
    uint8_t zone_letter_int;
    char zone_letter;

    if (!getInput("detected_objects", detected_objects)) {
        ROS_ERROR("[PublishPerceptionResults] Unable to obtain detected objects from blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("zone_number", zone_number)) {
        ROS_ERROR("[PublishPerceptionResults] Unable to obtain zone number from blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("zone_letter", zone_letter_int)) {
        ROS_ERROR("[PublishPerceptionResults] Unable to obtain zone letter from blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    zone_letter = (char) zone_letter_int;


    for (auto object : detected_objects) {
        if (object.name.find('[') == std::string::npos
            && std::find(tracker_ids.begin(), tracker_ids.end(), object.tracker_match_id[0]) == tracker_ids.end()) {
            geographic_msgs::GeoPoseStamped msg;

            msg.header.frame_id = object.name;
            msg.header.stamp = ros::Time::now();

            double x = object.world_coords[1];
            double y = object.world_coords[0];

            geographic_utils::utm_ned_to_latlon(x, y, zone_number, zone_letter, msg.pose.position);
            tracker_ids.push_back(object.tracker_match_id[0]);
            perception_task_pub_.publish(msg);
        }
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::PublishPerceptionResultsAction>("PublishPerceptionResults");
}
