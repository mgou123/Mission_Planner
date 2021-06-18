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

#ifndef MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TIME_EXPIRED_CONDITION_HPP_
#define MP_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_TIME_EXPIRED_CONDITION_HPP_

#include <string>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mp_behavior_tree
{

class IsTimeExpiredCondition : public BT::ConditionNode
{
public:
    IsTimeExpiredCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    IsTimeExpiredCondition() = delete;

    ~IsTimeExpiredCondition() override;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("seconds", 1.0, "Seconds")
        };
    }

protected:
    void cleanup() {}

private:
    std::shared_ptr<ros::NodeHandle> node_;
    ros::Time start_;
    double period_;
};

} // mp_behavior_tree

#endif