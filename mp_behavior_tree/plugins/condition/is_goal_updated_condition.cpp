// Copyright (c) 2020 Aitor Miguel Blanco
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
#include <vector>
#include "mp_behavior_tree/plugins/condition/is_goal_updated_condition.hpp"

namespace mp_behavior_tree
{

IsGoalUpdatedCondition::IsGoalUpdatedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus IsGoalUpdatedCondition::tick() {
    if (status() == BT::NodeStatus::IDLE) {
        getInput("goal", goal_);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::PoseStamped current_goal;
    getInput("goal", current_goal);

    if (goal_ != current_goal) {
        goal_ = current_goal;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

} //namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::IsGoalUpdatedCondition>("IsGoalUpdated");
}