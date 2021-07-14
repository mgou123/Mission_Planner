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

//wrapper class for wrapping a simple action client into a behavior tree action node

#ifndef MP_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define MP_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include "ros/ros.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "actionlib/client/simple_action_client.h"
#include "mp_behavior_tree/bt_conversions.hpp"

namespace mp_behavior_tree 
{
template<typename ActionT, typename GoalT, typename ResultT>
class BtActionNode : public BT::ActionNodeBase 
{
public:
    BtActionNode(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf) // for blackbaord and ports
        : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) 
    {
        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");

        // Initialise the input and output messages
        goal_ = GoalT();
        result_ = typename ResultT::ConstPtr();

        // Give the derive class a chance to do any initialization
        ROS_INFO("[%s] BtActionNode initialized", xml_tag_name.c_str());

    }

    BtActionNode() = delete;

    virtual ~BtActionNode() = default;

    void subscribe() {
        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        
        createActionClient(action_name_);

        ROS_INFO("[%s] BtActionNode subscribed to %s", name().c_str(), action_name_.c_str());
        subscribed_ = true;

    }
    
    // creates an instance of a simple action client
    void createActionClient(const std::string & action_name) 
    {
        action_client_ = std::make_shared<actionlib::SimpleActionClient<ActionT>>(*node_, action_name);

        // make sure the server is actually there before continuing
        ROS_INFO("Waiting for \"%s\" action server", action_name.c_str());
        action_client_->waitForServer();
    }

    // Any subclass of BtActionNode that accepts parameters must provide a providedPorts method
    // and call providedBasicPorts in it. 
    static BT::PortsList providedBasicPorts(BT::PortsList addition) 
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action server name"), // (name, description [optional])
            BT::InputPort<std::chrono::milliseconds>("server_timeout")
        };
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    // mandatory to define this method
    static BT::PortsList providedPorts() 
    {
        return providedBasicPorts({});
    }

    // Derived classes can override any of the following methods to hook into the
    // processing for the action: on_tick, on_wait_for_result, and on_success

    // Could do dynamic checks, such as getting updates to values on the blackboard
    virtual void on_tick() {}

    // There can be many loop iterations per tick. Any opportunity to do something after
    // a timeout waiting for a result that hasn't been received yet
    virtual void on_wait_for_result() {}

    // Called upon successful completion of the action. A derived class can override this
    // method to put a value on the blackboard, for example.
    virtual BT::NodeStatus on_success()
    {
        return BT::NodeStatus::SUCCESS;
    }

    // Called when a the action is aborted. By default, the node will return FAILURE.
    // The user may override it to return another value, instead.
    virtual BT::NodeStatus on_aborted()
    {
        return BT::NodeStatus::FAILURE;
    }

    // Called when the action is cancelled. By default, the node will return SUCCESS.
    // The user may override it to return another value, instead.
    virtual BT::NodeStatus on_cancelled()
    {
        return BT::NodeStatus::SUCCESS;
    }

    // The main override required by a BT action
    BT::NodeStatus tick() override
    {
        if (!subscribed_) {
            subscribe();
        }
       
        // this is to be done only at the beginning of the Action
        if (status() == BT::NodeStatus::IDLE) {
            setStatus(BT::NodeStatus::RUNNING);

            //user_defined callback
            on_tick();

            on_new_goal_received();
        }

        // the following code corresponds to the "RUNNING" loop
        if (ros::ok() && !goal_result_available_) {
            // user defined callback. May modify the value of "goal_updated_"
            on_wait_for_result();

            if (goal_updated_ && action_client_->getState() == actionlib::SimpleClientGoalState::ACTIVE) // The goal is currently being processed by the action server
            {
                goal_updated_ = false;
                on_new_goal_received();
            }

            return BT::NodeStatus::RUNNING;
        }

        auto state = action_client_->getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return on_success();
        } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
            return on_aborted();
        } else if (state == actionlib::SimpleClientGoalState::RECALLED) {
            return on_cancelled();
        } else {
            throw std::logic_error("BtActionNode::Tick: invalid status value");
        }
    }

    // The other (optional) override required by a BT action. In this case, we
    // make sure to cancel the action if it is still running.
    void halt() override
    {
        if (should_cancel_goal()) {
            action_client_ ->cancelAllGoals();
            if (!action_client_->waitForResult()) {
                ROS_ERROR("Failed to cancel action server for %s", action_name_.c_str());
            }
        }

        setStatus(BT::NodeStatus::IDLE);
    }

protected:
    bool should_cancel_goal() 
    {
        // only shut the node down if it is currently running
        if (status() != BT::NodeStatus::RUNNING) {
            return false;
        }

        // check if the goal is still executing, if so, cancel goal
        return action_client_->getState() == actionlib::SimpleClientGoalState::ACTIVE;
    }

    void on_new_goal_received()
    {
        goal_result_available_ = false;
        goal_active_ = false;

        action_client_->sendGoal(
            goal_,
            [this]( // done callback
                const actionlib::SimpleClientGoalState & state,
                const typename ResultT::ConstPtr & result) 
            {
                if (state == actionlib::SimpleClientGoalState::REJECTED) {
                    throw std::runtime_error("Goal was rejected by action server");
                }
                goal_result_available_ = true;
                result_ = result;
            },
            [this]() { // active callback
                goal_active_ = true;
            });
        
        // sleep until the goal is processed and rejected / active
        while (ros::ok() && !goal_active_) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void increment_recovery_count()
    {
        int recovery_count = 0;
        config().blackboard->get("number_recoveries", recovery_count);
        recovery_count += 1;
        config().blackboard->set("number_recoveries", recovery_count);
    }

    std::string action_name_;
    typename std::shared_ptr<actionlib::SimpleActionClient<ActionT>> action_client_;

    GoalT goal_;
    bool subscribed_{false};
    bool goal_updated_{false};
    bool goal_result_available_{false};
    bool goal_active_{false};

    typename ResultT::ConstPtr result_;

    std::shared_ptr<ros::NodeHandle> node_;
};


} // namespace mp_behavior_tree

#endif