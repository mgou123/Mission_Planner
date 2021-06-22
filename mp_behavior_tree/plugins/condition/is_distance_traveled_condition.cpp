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

#include <string>
#include <memory>

#include "mp_behavior_tree/plugins/condition/is_distance_traveled_condition.hpp"

namespace mp_behavior_tree
{

IsDistanceTraveledCondition::IsDistanceTraveledCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    distance_(1.0) 
{
    getInput("distance", distance_);
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
}

IsDistanceTraveledCondition::~IsDistanceTraveledCondition() {
    cleanup();
}

BT::NodeStatus IsDistanceTraveledCondition::tick()
{
    geometry_msgs::PoseStamped robot_pose;
    if (status() == BT::NodeStatus::IDLE) {
        if (!getInput("pose", start_pose_)) {
            ROS_WARN("[IsDistanceTraveled] No pose available");
        }
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", robot_pose)) {
        ROS_WARN("[IsDistanceTraveled] No pose available");
        return BT::NodeStatus::FAILURE;
    }

    double dx = start_pose_.pose.position.x - robot_pose.pose.position.x;
    double dy = start_pose_.pose.position.y - robot_pose.pose.position.y;
    double dz = start_pose_.pose.position.z - robot_pose.pose.position.z;

    if ((dx * dx + dy * dy + dz * dz) < (distance_ * distance_)) {
        return BT::NodeStatus::FAILURE;
    }

    start_pose_ = robot_pose;
    return BT::NodeStatus::SUCCESS;
}
} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsDistanceTraveledCondition>("IsDistanceTraveled");
}