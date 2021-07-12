#ifndef MP_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define MP_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace BT
{

// The following templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.

// Parse XML string to geometry_msgs::msg::Point
template<>
inline geometry_msgs::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

// Parse XML string to geometry_msgs::msg::Quaternion
template<>
inline geometry_msgs::Quaternion convertFromString(const StringView key)
{
  // four real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

// Parse XML string to geometry_msgs::msg::PoseStamped
template<>
inline geometry_msgs::PoseStamped convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';'); 
  if (parts.size() == 9) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  } else if (parts.size() == 7) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[0]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[1]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[6]);
    return pose_stamped;
  } else if (parts.size() == 4) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[0]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[1]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[2]);
    tf2::Quaternion quat;
    quat.setRPY(0, 0, BT::convertFromString<double>(parts[3]) / 180.0 * M_PI);
    quat = quat.normalize();
    tf2::convert(quat, pose_stamped.pose.orientation);
    return pose_stamped;
  } else if (parts.size() == 3) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[0]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[1]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    return pose_stamped;
  } else if (parts.size() == 2) { 
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[0]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[1]);
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    
    return pose_stamped;

  } else {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  }
}

// Parse XML string to std:;vector<std::string>
template<>
inline std::vector<std::string> convertFromString(const StringView key) 
{
  auto parts = BT::splitString(key, ',');
  std::vector<std::string> output;
  output.reserve(parts.size());
  for (const StringView &part : parts) {
      output.push_back(std::string(part.data(), part.size()));
  }

  return output;
}


} // namespace BT


#endif