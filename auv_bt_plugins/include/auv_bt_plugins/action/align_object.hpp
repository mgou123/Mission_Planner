#ifndef AUV_BT_PLUGINS__ACTION__ALIGN_OBJECT_HPP_
#define AUV_BT_PLUGINS__ACTION__ALIGN_OBJECT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace mp_behavior_tree {
  class AlignObject : public BT::SyncActionNode {
    public:
      AlignObject(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration &conf
      );

      ~AlignObject() {}

      static BT::PortsList providedPorts() {
        return {
          BT::InputPort<geometry_msgs::PoseStamped>("object", "Object to align to"),
          BT::InputPort<std::vector<std::string>>("zeroed_components", "Comma-separated. Takes in x,y,z"),
          BT::InputPort<geometry_msgs::PoseStamped>("modifying_pose", "Geometry Msg Pose to add to original"),
          BT::OutputPort<geometry_msgs::PoseStamped>("final_pose", "Geometry Msg Pose Final"), 
        };
      }

      BT::NodeStatus tick() override;
  };
}
#endif