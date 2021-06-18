#ifndef BT_MISSION_PLANNER__MISSION_PLANNER_HPP_
#define BT_MISSION_PLANNER__MISSION_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mp_behavior_tree/bt_action_server.hpp"
#include "mp_msgs/action/execute_mission.hpp"

namespace bt_mission_planner
{
class MissionPlanner
{
public:
    using MissionAction = mp_msgs::action::ExecuteMission; 
    MissionPlanner();

    ~Navigator();

    bool configure(ros::NodeHandle::WeakPtr parent_node);
    bool activate();
    bool deactivate();
    bool cleanup();
    bool shutdown();


private:
    std::vector<std::string> default_plugin_libs_;
    std::string goal_blackboard_id_;

    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener transform_listener_;

    std::unique_ptr<mp_behavior_tree::BtActionServer<MissionAction> bt_action_server_;
    bool onGoalReceived(MissionAction::Goal::ConstSharedPtr goal);
    void onCompletion(MissionAction::Result::SharedPtr result);
    void onLoop();
    void onPreempt(MissionAction::Goal::ConstSharedPtr goal);
    
};


} // namespace bt_mission_planner



#endif