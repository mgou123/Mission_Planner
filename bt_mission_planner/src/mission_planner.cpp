#include "bt_mission_planner/mission_planner.hpp"

#include <memory>
#include <string>
#include <vector>

#include <ros/package.h>

namespace bt_mission_planner
{
MissionPlanner::MissionPlanner()
{
    ROS_INFO("Creating mission planner...");
    default_plugin_libs_ = {};

    setParam("odom_topic", "/nav/odom/ned");
}

MissionPlanner::~MissionPlanner() {}

bool MissionPlanner::configure(ros::NodeHandle::WeakPtr parent_node) {
    ROS_INFO("Configuring mission planner...");
    auto node = parent_node.lock();

    tf_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);
    node->getParam("transform_tolerance", transform_tolerance_, 0.1);
    
    std::vector<std::string> plugin_lib_names;
    node->getParam("plugin_lib_names", plugin_lib_names, default_plugin_libs_);

    // get bt xml filename
    std::string default_bt_xml_filename;
    if (!node->hasParam("default_mission_planner_bt_xml")) {
        std::string path = ros::package::getPath("bt_mission_planner");
        std::string tree_file = path + "/behavior_trees/basic_demo.xml";
        node->setParam("default_mission_planner_bt_xml", tree_file);
    }

    node->getParam("default_mission_planner_bt_xml", default_bt_xml_filename);

    bt_action_server_ = std::make_unique<mp_behavior_tree::BtActionServer<MissionAction>>(
        node,
        "execute_mission",
        plugin_lib_names,
        default_bt_xml_filename,
        std::bind(&MissionPlanner::onGoalReceived, this, std::placeholders::_1),
        std::bind(&Navigator::onLoop, this),
        std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
        std::bind(&Navigator::onCompletion, this, std::placeholders::_1));
    
    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);

    node->setParam("goal_blackboard_id", "goal");
    node->getParam("goal_blackboard_id", goal_blackboard_id_);
    
    return ok;
}

bool MissionPlanner::activate() {
    ROS_INFO("Activating mission planner...");
    bool ok = true;
    if(!bt_action_server_.on_activate()) {
        ok = false;
    }

    return ok;
}

bool MissionPlanner::deactivate() {
    ROS_INFO("Deactivating mission planner...");
    bool ok = true;
    if(!bt_action_server_.on_deactivate()) {
        ok = false;
    }

    return ok;
}

bool MissionPlanner::cleanup() {
    ROS_INFO("Cleaning up mission planner...");
    tf_listener_.reset();
    tf_.reset();

    bool ok;
    if (!bt_action_server_->on_cleanup()) {
        ok = false;
    }

    bt_action_server_.reset();
    return ok;
}

bool MissionPlanner::shutdown() {
    ROS_INFO("Shutting down mission planner...");
    return true;
}

bool MissionPlanner::onGoalReceived(MissionAction::Goal::ConstSharedPtr goal) {
    auto bt_xml_filename = goal->behavior_tree;
    if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
        ROS_ERROR("BT file not found: %s. Mission canceled.", bt_xml_filename.c_str());
        return false;
    }

    ROS_INFO("Mission Planner goal received!");
    return true;
}

void MissionPlanner::onCompletion(MissionAction::Result::SharedPtr result) {}

void MissionPlanner::onLoop() {
    auto feedback_msg = std::make_shared<MissionAction::Feedback>();
    auto blackboard = bt_action_server_->getBlackboard();

    feedback_msg->current_task = "test";
    bt_action_server_->publishFeedback(feedback_msg);

}

void MissionPlanner::onPreempt(MissionAction::Goal::ConstSharedPtr goal) {
    ROS_INFO("Mission Planner goal preemption request received!");
    ROS_WARN("Preempt is currently not allowed. Please cancel the current goal and send a "
             "new action request instead. Continuing to execute previous goal...");
    bt_action_server_.terminatePendingGoal();
}

} // namespace bt_mission_planner