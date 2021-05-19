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

// publishes BT logs on BT status change

#ifndef MP_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
#define MP_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include <memory>
#include <utility>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "mp_msgs/msg/behavior_tree_log.h"
#include "mp_msgs/msg/behavior_tree_status_change.h"

namespace mp_behavior_tree
{
class BtTopicLogger : public BtStatusChangeLogger
{
public:
RosTopicLogger(const ros::NodeHandle::WeakPtr & parent, const BT::Tree & tree)
: StatusChangeLogger(tree.rootNode())
{
    auto node = parent.lock(); // creates a new shared_ptr that shares ownership of the managed object
    log_pub_ = node->advertise<mp_msgs::msg::BehaviorTreeLog>("bt_log", 10);
}

void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override 
{
    mp_msgs::msg::BehaviorTreeStatusChange event;
    event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
    event.node_name = node.name();
    event.previous_status = toStr(prev_status, false);
    event.current_status = toStr(status, false); 
    event_log_.push_back(std::move(event));
}

void flush() override
{
    if (!event_log_.empty()) {
        auto log_msg = std::make_unique<mp_msgs::msg::BehaviorTreeLog>();
        log_msg->header.stamp = ros::Time::now();
        log_msg->log = event_log_;
        log_pub_.publish(std::move(log_msg));
        event_log_.clear();
    }
}

protected:
    ros::Publisher log_pub_;
    std::vector<mp_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

} // namespace mp_behavior_tree


#endif // MP_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_