#include <string>

#include "asv_bt_plugins/action/navigate_to_geopose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace mp_behavior_tree
{

NavigateToGeopose::NavigateToGeopose(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<bb_msgs::LocomotionAction, 
                 bb_msgs::LocomotionGoal, 
                 bb_msgs::LocomotionResult>(xml_tag_name, action_name, conf) {}

void NavigateToGeopose::on_tick() {
    tf2::Quaternion quat;
    geographic_msgs::GeoPoseStamped currGeopose;
    geographic_msgs::GeoPoseStamped targetGeopose;

    if (!getInput("goal", targetGeopose)) {
        ROS_ERROR("[NavigateToGeopose] goal not provided!");
        return;
    }

    if (!getInput("origin", currGeopose)) {
        ROS_ERROR("[NavigateToGeopose] origin not provided!");
        return;
    }

    double enu[3];
    
    ROS_INFO(std::to_string(currGeopose.pose.position.latitude).c_str());
    ROS_INFO(std::to_string(currGeopose.pose.position.longitude).c_str());
    ROS_INFO(std::to_string(currGeopose.pose.position.altitude).c_str());

    ROS_INFO(std::to_string(targetGeopose.pose.position.latitude).c_str());
    ROS_INFO(std::to_string(targetGeopose.pose.position.longitude).c_str());
    ROS_INFO(std::to_string(targetGeopose.pose.position.altitude).c_str());

    this->GeodeticToEnu(targetGeopose.pose.position.latitude, targetGeopose.pose.position.longitude, targetGeopose.pose.position.altitude, 
                    currGeopose.pose.position.latitude, currGeopose.pose.position.longitude, currGeopose.pose.position.altitude,
                    enu[0], enu[1], enu[2]);

    ROS_INFO("Forward: %s", std::to_string(enu[1]).c_str());
    ROS_INFO("Sidemove: %s", std::to_string(enu[0]).c_str());

    geometry_msgs::PoseStamped goal_pose_stamped;
    double roll, pitch, yaw;
    bool rel;

    // set goal
    tf2::convert(targetGeopose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROS_INFO("Yaw: %s", std::to_string(yaw / M_PI * 180.0).c_str());

    // Goal is NED
    goal_.forward_setpoint = enu[1];
    goal_.sidemove_setpoint = enu[0];
    goal_.yaw_setpoint = 0; // yaw / M_PI * 180.0;

    goal_.movement_rel = true;
    goal_.yaw_rel = true;

    double sidemove_tolerance, forward_tolerance, yaw_tolerance;

    if (getInput("sidemove_tolerance", sidemove_tolerance)) {
      goal_.sidemove_tolerance = sidemove_tolerance;
    }

    if (getInput("forward_tolerance", forward_tolerance)) {
      goal_.forward_tolerance = forward_tolerance;
    }

    if (getInput("yaw_tolerance", yaw_tolerance)) {
      goal_.yaw_tolerance = yaw_tolerance;
    }

    setOutput("yaw_output", yaw / M_PI * 180.0);
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mp_behavior_tree::NavigateToGeopose>(
        name, "Locomotion", config);
    };

  factory.registerBuilder<mp_behavior_tree::NavigateToGeopose>(
    "NavigateToGeopose", builder);
}