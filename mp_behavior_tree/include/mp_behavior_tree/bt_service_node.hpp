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
#include <array>
#include <memory>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <stdexcept>


#include "ros/ros.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "mp_behavior_tree/bt_conversions.hpp"


namespace mp_behavior_tree
{

template <typename ServiceT>
class BtServiceNode : public BT::SyncActionNode
{
public:
    BtServiceNode(
        const std::string &xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration &conf)
      : BT::SyncActionNode(xml_tag_name, conf), service_name_(service_name)
    {
        node_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("node");

        //server_timeout_ = config().blackboard->get<std::chrono::milliseconds>("server_timeout");

        // Now that we have node_ to use, create the service client for this BT service
        std::string remapped_service_name;

        if (getInput("service_name", remapped_service_name)) {
            service_name_ = remapped_service_name;
        }

        service_client_ = std::make_shared<ros::ServiceClient>(node_->serviceClient<ServiceT>(service_name_));

        // Make a request for the service without parameter
        srv_ = std::make_shared<ServiceT>();

        // Make sure the server is actually there before continuing
        ROS_INFO("Waiting for \"%s\" service", service_name_.c_str());
        service_client_->waitForExistence();
        ROS_INFO("\"%s\" BtServiceNode initialized", xml_tag_name.c_str());
    }

    BtServiceNode() = delete;

    ~BtServiceNode() {};
    
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


    // The main override required by a BT service
    BT::NodeStatus tick() override
    {
        on_tick();
        if (yaml_req_.empty()) {
            if (!service_client_->call(*srv_)) {
                return on_failure();
            }

            return on_success();

        } else {
            std::array<char, 128> buffer;
            std::string cmd = "rosservice call " + service_name_ + " \"" + yaml_req_ + "\"";
            ROS_INFO("Running command: %s", cmd.c_str());
            std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
            if (!pipe) {
                ROS_ERROR("Unable to open pipe for service call!");
                return on_failure();
            }
            while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
                yaml_res_ += buffer.data();
            }
            
            return on_success();
        }
        
    }

    virtual void on_tick() = 0;

    virtual BT::NodeStatus on_success()
    {
        return BT::NodeStatus::SUCCESS;
    };

    virtual BT::NodeStatus on_failure()
    {
        return BT::NodeStatus::FAILURE;
    }


protected:
    std::string service_name_;
    std::shared_ptr<ros::ServiceClient> service_client_;
    std::shared_ptr<ServiceT> srv_;
    std::string yaml_req_;
    std::string yaml_res_;

    // The node that will be used for any ROS operations
    std::shared_ptr<ros::NodeHandle> node_;

};

} // namespace mp_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_