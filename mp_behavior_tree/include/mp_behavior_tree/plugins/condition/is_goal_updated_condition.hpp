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


#ifndef MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_UPDATED_CONDITION_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/PoseStamped.h"

namespace mp_behavior_tree
{
class IsGoalUpdatedCondition : public BT::ConditionNode
{
public:
    IsGoalUpdatedCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    ~IsGoalUpdatedCondition() {};

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination")
        };
    }

private:
    geometry_msgs::PoseStamped goal_;    
};

} // namespace mp_behavior_tree


#endif