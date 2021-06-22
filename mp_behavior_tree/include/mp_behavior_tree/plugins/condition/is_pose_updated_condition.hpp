#ifndef MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_POSE_UPDATED_CONDITION_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_POSE_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/PoseStamped.h"

namespace mp_behavior_tree
{
class IsPoseUpdatedCondition : public BT::ConditionNode
{
public:
    IsPoseUpdatedCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    IsPoseUpdatedCondition() = delete;

    ~IsPoseUpdatedCondition() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("pose", "Latest robot pose")
        };
    }


protected:
    void cleanup() {};

private:
    geometry_msgs::PoseStamped pose_;
};

} // namespace mp_behavior_tree

#endif