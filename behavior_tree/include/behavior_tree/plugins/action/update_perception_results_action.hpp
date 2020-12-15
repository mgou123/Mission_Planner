// Updates blackboard with vision pipeline results

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_PERCEPTION_RESULTS_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_PERCEPTION_RESULTS_ACTION_HPP_

#include "ros/ros.h"
#include "asv_msgs/DetectedObjects.h"

#include "behaviortree_cpp_v3/action_node.h"

namespace behavior_tree
{
class UpdatePerceptionResultsAction : public BT::ActionNodeBase
{
public:
    UpdatePerceptionResultsAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    UpdatePerceptionResultsAction() = delete;
    BT::NodeStatus tick() override;
    void halt() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("detected_topic", "Topic name for detected objects"),
            BT::OutputPort<std::vector<asv_msgs::DetectedObject>>("detected_objects", "Array of detected objects")
        };
    }

private:
    void detectedCallback(const asv_msgs::DetectedObjectsConstPtr & msg);
    
    std::shared_ptr<ros::NodeHandle> node_;
    std::string detected_topic_;
    ros::Subscriber detected_sub_;
    std::vector<asv_msgs::DetectedObject> detected_objects_;
};

} // namespace behavior_tree

#endif