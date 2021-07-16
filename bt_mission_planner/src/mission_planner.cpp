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
}

MissionPlanner::~MissionPlanner() {}

bool MissionPlanner::configure(std::weak_ptr<ros::NodeHandle> parent_node) {

    ROS_INFO("Configuring mission planner...");
    auto node = parent_node.lock();

    tf_ = std::make_shared<tf2_ros::Buffer>();
    
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    if (!node->hasParam("transform_tolerance")) {
        node->setParam("transform_tolerance", 0.1);
    }
    if (!node->hasParam("transform_timeout")) {
        node->setParam("transform_timeout", 1.0);
    }

    std::vector<std::string> plugin_lib_names;
    if (!node->hasParam("plugin_lib_names")) {
        node->setParam("plugin_lib_names", default_plugin_libs_);
    }

    std::string odom_topic;
    if (!node->hasParam("odom_topic")) {
        node->setParam("odom_topic", "/nav/odom_ned");
    }

    // get bt xml filename
    std::string default_bt_xml_filename;

    if (node->getParam("default_mission_planner_bt_xml", default_bt_xml_filename)) {
        if (default_bt_xml_filename.at(0) != '/') {
            std::string path = ros::package::getPath("bt_mission_planner");
            std::string tree_file = path + "/behavior_trees/" + default_bt_xml_filename;
            node->setParam("default_mission_planner_bt_xml", tree_file);
        }
    } else {
        std::string path = ros::package::getPath("bt_mission_planner");
        std::string tree_file = path + "/behavior_trees/basic_demo.xml";
        node->setParam("default_mission_planner_bt_xml", tree_file);
    }

    node->getParam("plugin_lib_names", plugin_lib_names);
    node->getParam("default_mission_planner_bt_xml", default_bt_xml_filename);

    bt_action_server_ = std::make_unique<mp_behavior_tree::BtActionServer<
                            MissionAction, MissionGoal, MissionResult, MissionFeedback>>(
        node,
        "execute_mission",
        plugin_lib_names,
        default_bt_xml_filename,
        std::bind(&MissionPlanner::onGoalReceived, this, std::placeholders::_1),
        std::bind(&MissionPlanner::onLoop, this),
        std::bind(&MissionPlanner::onPreempt, this, std::placeholders::_1),
        std::bind(&MissionPlanner::onCompletion, this, std::placeholders::_1));

    std::cout << default_bt_xml_filename << std::endl;
    for (auto i = plugin_lib_names.begin(); i != plugin_lib_names.end(); ++i)
        std::cout << *i << ' ';
        
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
    if(!bt_action_server_->on_activate()) {
        ok = false;
    }

    return ok;
}

bool MissionPlanner::deactivate() {
    ROS_INFO("Deactivating mission planner...");
    bool ok = true;
    if(!bt_action_server_->on_deactivate()) {
        ok = false;
    }

    return ok;
}

bool MissionPlanner::cleanup() {
    ROS_INFO("Cleaning up mission planner...");
    transform_listener_.reset();
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

bool MissionPlanner::onGoalReceived(const MissionGoal::ConstPtr &goal) {
    auto bt_xml_filename = goal->behavior_tree;
    if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
        ROS_ERROR("BT file not found: %s. Mission canceled.", bt_xml_filename.c_str());
        return false;
    }

    ROS_INFO("Mission Planner goal received!");
    return true;
}

void MissionPlanner::onCompletion(const MissionResult::ConstPtr &result) {}

void MissionPlanner::onLoop() {
    auto feedback_msg = std::make_shared<MissionFeedback>();
    auto blackboard = bt_action_server_->getBlackboard();

    // feedback_msg->current_task = "test";
    // bt_action_server_->publishFeedback(feedback_msg);
}

void MissionPlanner::onPreempt(const MissionGoal::ConstPtr &goal) {
    ROS_INFO("Mission Planner goal preemption request received!");
    ROS_WARN("Preempt is currently not allowed. Please cancel the current goal and send a "
             "new action request instead. Continuing to execute previous goal...");
    //bt_action_server_->terminatePendingGoal();
}

} // namespace bt_mission_planner