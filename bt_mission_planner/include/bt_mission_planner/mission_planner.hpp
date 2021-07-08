#ifndef BT_MISSION_PLANNER__MISSION_PLANNER_HPP_
#define BT_MISSION_PLANNER__MISSION_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mp_behavior_tree/bt_action_server.hpp"
#include "mp_behavior_tree/bt_conversions.hpp"

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
    using MissionFeedback = mp_msgs::ExecuteMissionFeedback;

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

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    std::unique_ptr<mp_behavior_tree::BtActionServer<
        MissionAction, MissionGoal, MissionResult, MissionFeedback>> bt_action_server_;
    bool onGoalReceived(const MissionGoal::ConstPtr &goal);
    void onCompletion(const MissionResult::ConstPtr &result);
    void onLoop();
    void onPreempt(const MissionGoal::ConstPtr &goal);
    
};


} // namespace bt_mission_planner



#endif