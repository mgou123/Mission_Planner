// Copyright (c) 2020 Sarthak Mittal
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

//action server that uses behaviour trees to execute an action

#ifndef MP_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define MP_BEHAVIOR_TREE__BT_ACTION__SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "mp_behavior_tree/bt_action_server.hpp"

namespace mp_behavior_tree
{
template<class ActionT<
BtActionServer<ActionT>:BtActionServer(
    const ros::NodeHandle::WeakPtr & parent,
    const std::string & action_name,
    const std::vector<std::string> & plugin_lib_names,
    const std::string & default_bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback,
    OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback)
  : action_name_(action_name),
    default_bt_xml_filename_(default_bt_xml_filename),
    plugin_lib_names_(plugin_lib_names),
    node_(parent),
    on_goal_received_callback_(on_goal_received_callback),
    on_loop_callback_(on_loop_callback),
    on_preempt_callback_(on_preempt_callback),
    on_completion_callback_(on_completion_callback)
{
    auto node = node_.lock();

    // Declare node parameters
    if (!node->hasParam("bt_loop_duration")) {
        node->setParam("bt_loop_duration", 10);
    }

    if (!node->hasParam("default_server_timeout")) {
        node->setParam("default_server_timeout", 20);
    }

    if (!node->hasParam("enable_groot_monitoring")) {
        node->setParam("enable_groot_monitoring", true);
    }

    if (!node->hasParam("groot_zmq_publisher_port")) {
        node->setParam("groot_zmq_publisher_port", 1666);
    }

    if (!node->hasParam("groot_zmq_server_port")) {
        node->setParam("groot_zmq_server_port", 1667);
    }
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer() {}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure() 
{
    auto node = node_.lock();
    if (!node) {
        throw sd::runtime_error("Failed to lock node");
    }

    client_node_ = std::make_shared<ros::NodeHandle>("_");
    action_server_ = std::make_shared<ActionServer>(
        *node, action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this), false);
    
    // Get parameter for monitoring with Groot via ZMQ Publisher
    node->getParam("enable_groot_monitoring", enable_groot_monitoring_);
    node->getParam("groot_zmq_publisher_port", groot_zmq_publisher_port_);
    node->getParam("groot_zmq_server_port", groot_zmq_server_port_);

    // Get parameters for BT timeouts
    int timeout;
    node->getParam("bt_loop_duration", timeout);
    bt_loop_duration_ = std::chrono::milliseconds(timeout);
    node->getParam("default_server_timeout", timeout);
    default_server_timeout_ = std::chrono::milliseconds(timeout);

    // Create the class that registers our custom nodes and executes the BT
    bt_ = std::make_unique<mp_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);
    
    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard_->set<ros::NodeHandle::SharedPtr>("node", client_node_);
    blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);

    return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
    if (!loadBehaviorTree(default_bt_xml_filename_)) {
        ROS_ERROR("Error loading XML file: %s", default_bt_xml_filename_.c_str());
        return false;
    }

    action_server_->start();
    return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
    action_server_->shutdown();
    return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup() 
{
    client_node_.reset();
    action_server_.reset();
    topic_logger_.reset();
    plugin_lib_names_.clear();
    current_bt_xml_filename_.clear();
    blackboard_.reset();

    bt_->haltAllActions(tree_.rootNode());
    bt_->resetGrootMonitor();
    bt_.reset();

    return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
    auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    // Use previous BT if it is the existing one
    if (current_bt_xml_filename_ == filename) {
        ROS_DEBUG("BT will not be reloaded as the given XML is already loaded");
        return true;
    }

    // if a new tree is created, then the current ZMQ Publisher must be destroyed
    bt_->resetGrootMonitor();

    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(filename);

    if (!xml_file.good()) {
        ROS_ERROR("Couldn't open input XML file: %s", filename.c_str());
        return false;
    }

    auto xml_string = std::string(
        std::istreambuf_iterator<char>(xml_file);
        std::istreambuf_iterator<char>());

    // Create behavior tree from XML input
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);
    topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

    current_bt_xml_filename_ = filename;

    // Enable monitoring with Groot
    if (enable_groot_monitoring_) {
        try {
            bt_->addGrootMonitoring(&tree_, groot_zmq_publisher_port_, groot_zmq_server_port_);
        } catch (const std::logic_error & e) {
            ROS_ERROR("ZMQ already enabled, Error: %s", e.what());
        }
    }

    return true;
}

template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
    if (!on_goal_received_callback_(action_server_)) {
        action_server_->setPreempted();
        return;
    }

    auto is_canceling = [&]() {
        if (action_server_ == nullptr) {
            ROS_DEBUG("Action server unavailable. Canceling.");
            return true;
        }
        if (!action_server_->isActive()) {
            ROS_DEBUG("Action server i inactive. Canceling.");
            return true;
        }
        return false;
    };

    auto on_loop = [&]() {
        if (action_server_->isPreemptRequested() && on_preempt_callback_) {
            on_preempt_callback_(action_server_);
        }
        topic_logger_.flush();
        on_loop_callback_();
    };

    // Execute the BT that was previously created in the configure step
    mp_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

    // Make sure that the Bt is not in a running state from a previous execution
    bt_->haltAllActions(tree_.rootNode());

    // Populate result message
    auto result = std::make_shared<typename ActionT::Result>();
    on_completion_callback_(result);

    switch (rc) {
        case mp_behavior_tree::BtStatus::SUCCEEDED:
            ROS_INFO("Goal succeeded");
            action_server_->setSucceeded(result);
            break;
        case mp_behavior_tree::BtStatus::FAILED:
            ROS_ERROR("Goal failed");
            action_server_->setAborted(result);
            break;
        case mp_behavior_tree->CANCELED:
            ROS_INFO("Goal canceled");
            action_server_->setPreempted(result);
            break;
    }
}

} // namespace mp_behavior_tree

#endif