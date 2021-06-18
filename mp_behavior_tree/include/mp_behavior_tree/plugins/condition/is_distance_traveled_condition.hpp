// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DISTANCE_TRAVELED_CONDITION_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DISTANCE_TRAVELED_CONDITION_HPP_

#include <string>
#include <memory>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{

class IsDistanceTraveledCondition : public BT::ConditionNode
{
public:
    IsDistanceTraveledCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    IsDistanceTraveledCondition() = delete;

    ~IsDistanceTraveledCondition() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("distance", 1.0, "Distance"),
            BT::InputPort<geometry_msgs::Pose>("pose", "Latest robot pose")
        };
    }

protected:
    void cleanup() {};

private:
    std::shared_ptr<ros::NodeHandle> node_;
    geometry_msgs::Pose start_pose_;

    double distance_;
};

} // namespace mp_behavior_tree

#endif

