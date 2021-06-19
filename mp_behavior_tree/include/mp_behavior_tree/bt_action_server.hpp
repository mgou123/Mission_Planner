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

#ifndef MP_BEHAVIOR_TREE__BT_ACTION_SERVER_NODE_HPP_
#define MP_BEHAVIOR_TREE__BT_ACTION_SERVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include "mp_behavior_tree/behavior_tree_engine.hpp"
#include "mp_behavior_tree/ros_topic_logger.hpp"

namespace mp_behavior_tree
{
template <typename ActionT, typename GoalT, typename ResultT, typename FeedbackT>
class BtActionServer
{
public:
    using ActionServer = actionlib::SimpleActionServer<ActionT>;

    typedef std::function<bool (const typename GoalT::ConstPtr)> OnGoalReceivedCallback;
    typedef std::function<void ()> OnLoopCallback;
    typedef std::function<void (const typename GoalT::ConstPtr)> OnPreemptCallback;
    typedef std::function<void (const typename ResultT::ConstPtr)> OnCompletionCallback;

    explicit BtActionServer(
        const std::weak_ptr<ros::NodeHandle> & parent,
        const std::string & action_name,
        const std::vector<std::string> & plugin_lib_names,
        const std::string & default_bt_xml_filename,
        OnGoalReceivedCallback on_goal_received_callback,
        OnLoopCallback on_loop_callback,
        OnPreemptCallback on_preempt_callback,
        OnCompletionCallback on_completion_callback
    );

    ~BtActionServer();

    bool on_configure();
    bool on_activate();
    bool on_deactivate();
    bool on_cleanup();

    bool loadBehaviorTree(const std::string & bt_xml_filename = "");
    
    BT::Blackboard::Ptr getBlackboard() const
    {
        return blackboard_;
    }

    std::string getCurrentBTFilename() const
    {
        return current_bt_xml_filename_;
    }

    std::string getDefaultBTFilename() const
    {
        return default_bt_xml_filename_;
    }

    BT::Tree getTree() const
    {
        return tree_;
    }

    // wrapper function to accept pending goal
    typename GoalT::ConstPtr acceptPendingGoal()
    {
        return action_server_->acceptNewGoal();
    }

    // wrapper function to publish feedback
    void publishFeedback(typename FeedbackT::ConstPtr feedback) 
    {
        action_server_->publishFeedback(feedback);
    }

    // Function to halt the current tree. It will interrupt the execution of RUNNING nodes
    // by calling their halt() implementation (only for Async nodes that may return RUNNING)
    void haltTree()
    {
        tree_.rootNode()->halt();
    }

protected:
    void executeCallback(const typename GoalT::ConstPtr &goal);
    std::string action_name_;
    std::shared_ptr<ActionServer> action_server_;

    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    std::unique_ptr<mp_behavior_tree::BehaviorTreeEngine> bt_;
    std::vector<std::string> plugin_lib_names_;

    // A regular, non-spinning ROS node that we can use for calls to the action client
    std::shared_ptr<ros::NodeHandle> client_node_;

    // Parent node
    std::weak_ptr<ros::NodeHandle> node_;
    
    // To publish BT logs
    std::unique_ptr<BtTopicLogger> topic_logger_;

    double bt_loop_duration_;
    double default_server_timeout_;

    // Parameters for Groot monitoring
    bool enable_groot_monitoring_;
    int groot_zmq_publisher_port_;
    int groot_zmq_server_port_;

    // User-provided callbacks
    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;

};

} // namespace mp_behavior_tree

#include <mp_behavior_tree/bt_action_server_impl.hpp>
#endif // MP_BEHAVIOR_TREE__BT_ACTION_SERVER_NODE_HPP_