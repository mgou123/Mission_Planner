#include <memory>
#include <string>

#include "auv_bt_plugins/action/navigate_control.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace mp_behavior_tree
{

  NavigateControl::NavigateControl(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<bb_msgs::LocomotionAction,
                     bb_msgs::LocomotionGoal,
                     bb_msgs::LocomotionResult>(xml_tag_name, action_name, conf) {}

  void NavigateControl::on_tick()
  {
    tf2::Quaternion quat;
    float forward, sideways, depth, yaw;

    double x, y, z;
    bool movement_rel = true;

    if (!getInput("forward", forward))
    {
      ROS_WARN("[NavigateControl] forward not provided! Station-keeping!");
      forward = 0;
    }

    if (!getInput("sideways", sideways))
    {
      ROS_WARN("[NavigateControl] sideways not provided! Station-keeping!");
      sideways = 0;
    }

    if (!getInput("depth", depth))
    {
      ROS_WARN("[NavigateControl] depth not provided! Station-keeping!");
      depth = 0;
    }

    if (!getInput("yaw", yaw))
    {
      ROS_WARN("[NavigateControl] yaw not provided! Station-keeping!");
      yaw = 0;
    }

    ROS_WARN("[NavigateControl] Sending to Controls!");
    // Using NED Convention
    goal_.forward_setpoint = forward;
    goal_.sidemove_setpoint = sideways;
    goal_.depth_setpoint = depth;
    goal_.yaw_setpoint = yaw;

    // Setting up relative motion
    goal_.yaw_rel = true;
    goal_.movement_rel = true;
    goal_.controller_state = 0;
  }

  void NavigateControl::on_wait_for_result()
  {
  }

} // namespace mp_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<mp_behavior_tree::NavigateControl>(
        name, "Locomotion", config);
  };

  factory.registerBuilder<mp_behavior_tree::NavigateControl>(
      "NavigateControl", builder);
}