#ifndef VISION_BT_PLUGINS__CONDITIOn__IS_OBJECT_DETECTED_CONDITION_HPP_
#define VISION_BT_PLUGINS__CONDITIOn__IS_OBJECT_DETECTED_CONDITION_HPP_


#include "ros/ros.h"
#include "bb_msgs/DetectedObjects.h"
#include "bb_msgs/DetectedObject.h"

#include "geometry_msgs/PoseStamped.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{
class IsObjectDetectedCondition : public BT::ConditionNode
{
public:
    IsObjectDetectedCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    ~IsObjectDetectedCondition() {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("pose", "Latest robot pose"),
            BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination"),
        };
    }
    


};



}

#endif