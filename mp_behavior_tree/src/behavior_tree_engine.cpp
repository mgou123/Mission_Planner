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

#include "mp_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace std::chrono_literals;

namespace mp_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(
    const std::vector<std::string> & plugin_libraries,
    bool enable_text_logging,
    bool enable_groot_logging, 
    const std::string bt_logging_filename)
{
    enable_text_logging_ = enable_text_logging;
    enable_groot_logging_ = enable_groot_logging;
    bt_logging_filename_ = bt_logging_filename;

    for (const auto & p : plugin_libraries) {
        factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    }
}

BtStatus BehaviorTreeEngine::run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    double loopFrequency)
{
    ros::Rate loopRate(loopFrequency);

    std::shared_ptr<BT::StdCoutLogger> logger_cout_ptr;
    std::shared_ptr<BT::FileLogger> logger_file_ptr;

    if (enable_text_logging_) {
        ROS_INFO("Enabling text logging...");
        logger_cout_ptr = std::make_shared<BT::StdCoutLogger>(*tree);
    }
    
    if (enable_groot_logging_) {
        ROS_INFO("Enabling goot logging...");
        logger_file_ptr = std::make_shared<BT::FileLogger>(*tree, bt_logging_filename_.c_str());
    }
    
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    // Loop until something happens with ROS or the node completes
    while (ros::ok() && result == BT::NodeStatus::RUNNING) {
        if (cancelRequested()) {
            tree->rootNode()->halt();
            return BtStatus::CANCELED;
        }

        result = tree->tickRoot();

        onLoop();

        loopRate.sleep();
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromFile(file_path, blackboard);
}

void
BehaviorTreeEngine::addGrootMonitoring(BT::Tree * tree)
{
    // This logger publish status changes using ZeroMQ. Used by Groot
    publisher_zmq_ = std::make_unique<BT::PublisherZMQ>(*tree);
}

void
BehaviorTreeEngine::resetGrootMonitor()
{
    publisher_zmq_.reset();
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
    // this halt signal should propagate through the entire tree.
    root_node->halt();

    // but, just in case...
    auto visitor = [](BT::TreeNode * node) {
        if (node->status() == BT::NodeStatus::RUNNING) {
            node->halt();
        }
    };
    BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace mp_behavior_tree