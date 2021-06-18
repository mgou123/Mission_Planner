#ifndef BT_MISSION_PLANNER__MISSION_PLANNER_HPP_
#define BT_MISSION_PLANNER__MISSION_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mp_behavior_tree/bt_action_server.hpp"
#include "mp_msgs/ExecuteMissionAction.h"
#include "mp_msgs/ExecuteMissionGoal.h"
#include "mp_msgs/ExecuteMissionResult.h"

namespace bt_mission_planner
{
class MissionPlanner
{
public:
    using MissionAction = mp_msgs::ExecuteMissionAction; 
    using MissionGoal = mp_msgs::ExecuteMissionGoal;
    using MissionResult = mp_msgs::ExecuteMissionResult;

    MissionPlanner();

    ~MissionPlanner();

    bool configure(std::weak_ptr<ros::NodeHandle> parent_node);
    bool activate();
    bool deactivate();
    bool cleanup();
    bool shutdown();


private:
    std::vector<std::string> default_plugin_libs_;
    std::string goal_blackboard_id_;

    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener transform_listener_;

    std::unique_ptr<mp_behavior_tree::BtActionServer<MissionAction>> bt_action_server_;
    bool onGoalReceived(const std::shared_ptr<MissionGoal> goal);
    void onCompletion(const std::shared_ptr<MissionResult> result);
    void onLoop();
    void onPreempt(const std::shared_ptr<MissionGoal> goal);
    
};


} // namespace bt_mission_planner



#endif