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
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus NavigateToGeopose::tick() {
    tf2::Quaternion quat;
    geographic_msgs::GeoPoseStamped currGeopose;
    geographic_msgs::GeoPoseStamped targetGeopose;

    geometry_msgs::PoseStamped currPose;

    if (!getInput("goal", targetGeopose)) {
        ROS_ERROR("[NavigateToGeopose] goal not provided!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("origin", currGeopose)) {
        ROS_ERROR("[NavigateToGeopose] origin not provided!");
        return BT::NodeStatus::FAILURE;
    }

    
    if (!getInput("curr_pose", currPose)) {
        ROS_ERROR("[NavigateToGeopose] current pose not provided!");
        return BT::NodeStatus::FAILURE;
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

    // Translation to relative
    geometry_msgs::Vector3 local_geo;
    local_geo.x = enu[0];
    local_geo.y = enu[1];
    local_geo.z = enu[2];

    tf2::Quaternion currQuat;
    tf2::Vector3 temp_vec;
    tf2::convert(currPose.pose.orientation, currQuat);
    tf2::convert(local_geo, temp_vec);
    temp_vec = tf2::quatRotate(currQuat, temp_vec);
    tf2::convert(temp_vec, local_geo);

    enu[0] = local_geo.x;
    enu[1] = local_geo.y;
    enu[2] = local_geo.z;

    // Translation finished

    geometry_msgs::PoseStamped goal_pose_stamped;
    double roll, pitch, yaw;
    bool rel;

    // set goal
    tf2::convert(targetGeopose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    yaw = 90.0 + yaw / M_PI * 180.0; // This is absolute yaw; the 90 degrees is to offset some frame change
    ROS_INFO("Abs. Yaw: %s", std::to_string(yaw).c_str());

    // Goal is NED
    goal_pose_stamped.pose.position.x = enu[1];
    goal_pose_stamped.pose.position.y = enu[0];
    goal_pose_stamped.pose.position.z = 0;

    setOutput("yaw_output", yaw); // Absolute yaw
    setOutput("pose_output", goal_pose_stamped); // Relative pose

    return BT::NodeStatus::SUCCESS;
}

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mp_behavior_tree::NavigateToGeopose>("NavigateToGeopose");
}
                    