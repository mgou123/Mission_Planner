// Copyright (c) 2019 Samsung Research America
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

#ifndef MP_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define MP_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>
#include <cstdlib>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace mp_behavior_tree
{

template <typename ServiceT>
class BtServiceNode : public BT::SyncActionNode
{
public:
    BtServiceNode(
        const std::string &service_node_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(service_node_name, conf), service_node_name_(service_node_name)
    {
        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");

        //server_timeout_ = config().blackboard->get<std::chrono::milliseconds>("server_timeout");

        // Now that we have node_ to use, create the service client for this BT service
        getInput("service_name", service_name_);
        service_client_ = std::make_shared<node_->serviceClient<ServiceT>>(service_name_);

        // Make a request for the service without parameter
        request_ = std::make_shared<typename ServiceT::Request>();
        response_ = std::make_shared<typename ServiceT::Response>();

        // Make sure the server is actually there before continuing
        ROS_INFO("Waiting for \"%s\" service", service_name_.c_str());
        service_client_->waitForExistence();
        ROS_INFO("\"%s\" BtServiceNode initialized", service_node_name_.c_str());
    }

    BtServiceNode() = delete;

    virtual ~BtServiceNode() = default;
    
    // Any subclass of BtServiceNode that accepts parameters must provide a providedPorts method
    // and call providedBasicPorts in it.
    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("service_name", "Service name for BT node"),
            //BT::InputPort<std::chrono::milliseconds>("server_timeout", 100, "timeout to connect to server (milliseconds)")
        };
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    virtual void set_request() = 0;

    virtual BT::NodeStatus on_success() = 0;

    virtual BT::NodeStatus on_failure()
    {
        return BT::NodeStatus::FAILURE;
    }

    // The main override required by a BT service
    BT::NodeStatus tick() override
    {
        set_request();
        if (!service_client_.call(resquest_, response_)) {
            return on_failure();
        }

        return on_success();
    }

protected:
    void increment_recovery_count()
    {
        int recovery_count = 0;
        config().blackboard->get("number_recoveries", recovery_count); // NOLINT
        recovery_count += 1;
        config().blackboard->set("number_recoveries", recovery_count); // NOLINT
    }

    std::string service_name_, service_node_name_;
    typename std::shared_ptr<ros::ServiceClient<ServiceT>> service_client_;
    std::shared_ptr<typename ServiceT::Request> request_;
    std::shared_ptr<typename ServiceT::Response> response_;

    // The node that will be used for any ROS operations
    std::shared_ptr<ros::NodeHandle> node_;
};

} // namespace mp_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_