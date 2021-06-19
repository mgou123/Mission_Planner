#include <memory>

#include <ros/ros.h>
#include "bt_mission_planner/mission_planner.hpp"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mission_planner");
    std::shared_ptr<ros::NodeHandle> node;

    auto mp = std::make_shared<bt_mission_planner::MissionPlanner>();

    if (!mp->configure(node)) {
        ROS_ERROR("Unable to configure mission planner! Aborting...");
        return 0;
    };

    if (!mp->activate()) {
        ROS_ERROR("Unable to activate mission planner! Aborting...");
        return 0;
    }
    
    ros::Rate r(10); // 10 hz

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    
    if (!mp->deactivate() && mp->cleanup()) {
        ROS_WARN("Mission planner not properly deactivated and cleaned up! There may be remaining resources taken up.");

    };
    
    mp->shutdown();

    return 0;
}