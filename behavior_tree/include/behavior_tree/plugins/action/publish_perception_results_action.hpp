// publish perception results for VORC perception task

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_PERCEPTION_RESULTS_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_PERCEPTION_RESULTS_ACTION_HPP_

#include <ros/ros.h>

#include "geographic_msgs/GeoPoseStamped.h"
#include "asv_msgs/DetectedObject.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{

class PublishPerceptionResultsAction : public BT::SyncActionNode
{
public:
    PublishPerceptionResultsAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    PublishPerceptionResultsAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("perception_task_topic", "Topic for publishing results for perception task"),
            BT::InputPort<int>("zone_number", "UTM zone number of latest robot pose"),
            BT::InputPort<uint8_t>("zone_letter", "UTM zone letter of latest robot pose"),
            BT::InputPort<std::vector<asv_msgs::DetectedObject>>("detected_objects", "Array of detected objects")
        };
    }

private:
    std::shared_ptr<ros::NodeHandle> node_;
    std::string perception_task_topic_;
    ros::Publisher perception_task_pub_;
    std::vector<uint32_t> tracker_ids;
};



} // namespace behavior_tree

#endif

