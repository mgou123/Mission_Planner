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

#include "behaviortree_cpp_v3/condition_node.h"

#include "mp_behavior_tree/plugins/condition/is_time_expired_condition.hpp"

namespace mp_behavior_tree
{
IsTimeExpiredCondition::IsTimeExpiredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    period_(1.0) {
    
    getInput("seconds", period_);
    node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");
    start_ = ros::Time::now();
}

IsTimeExpiredCondition::~IsTimeExpiredCondition()
{
  cleanup();
}

BT::NodeStatus IsTimeExpiredCondition::tick() {
    if (status() == BT::NodeStatus::IDLE) {
        start_ = ros::Time::now();
        return BT::NodeStatus::FAILURE;
    }

    double elapsed = (ros::Time::now() - start_).toSec();
    if (elapsed < period_) {
        return BT::NodeStatus::FAILURE;
    }

    start_ = ros::Time::now();
    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsTimeExpiredCondition>("IsTimeExpired");
}

