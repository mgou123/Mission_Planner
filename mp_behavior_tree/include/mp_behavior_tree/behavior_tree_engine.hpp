// Copyright (c) 2018 Intel Corporation
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

#ifndef MP_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define MP_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <ros/duration.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

namespace mp_behavior_tree
{

enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

class BehaviorTreeEngine
{
public:
    explicit BehaviorTreeEngine(
        const std::vector<std::string> & plugin_libraries,
        bool enable_text_logging = false,
        bool enable_groot_logging = false,
        const std::string bt_logging_filename = "bt_trace.fbl");
    virtual ~BehaviorTreeEngine() {}

    BtStatus run(
        BT::Tree * tree,
        std::function<void()> onLoop,
        std::function<bool()> cancelRequested,
        double loopTimeout = 100.0);
    
    BT::Tree createTreeFromText(
        const std::string & xml_string,
        BT::Blackboard::Ptr blackboard);

    BT::Tree createTreeFromFile(
        const std::string & file_path,
        BT::Blackboard::Ptr blackboard);
    
    void addGrootMonitoring(BT::Tree * tree);
    void resetGrootMonitor();

    // In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
    void haltAllActions(BT::TreeNode * root_node);

protected:
    // The factory that will be used to dynamically construct the behavior tree
    BT::BehaviorTreeFactory factory_;

    // Groot visualization
    std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;

    bool enable_groot_logging_;
    bool enable_text_logging_;
    std::string bt_logging_filename_;
};

} // namespace mp_behavior_tree

#endif