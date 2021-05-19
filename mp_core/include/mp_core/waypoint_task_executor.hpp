#ifndef MP_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define MP_CORE__WAYPOINT_TASK_EXECUTOR_HPP_

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mp_core
{
/**
 * @class mp_core::WaypointTaskExecutor
 * @brief This class defines the plugin interface for performing
 * specific tasks at waypoint arrivals
 */
class WaypointTaskExecutor
{
public:
    typedef std::shared_ptr<mp_core::WaypointTaskExecutor> Ptr;

    virtual ~WaypointTaskExecutor() {}

    /**
     * @brief Initialize parameters for WaypointTaskExecutor
     * @param parent Node pointer to parent that plugin will be created within
     * @param plugin_name Plugin name comes from parameters in yaml file
     */
    virtual void initialize(
        const ros::NodeHandle & parent,
        const std::string & plugin_name) = 0;
    
    /**
     * @brief Defines body of the task to be executed once the robot arrives
     * at the waypoint
     * @param current_pose Current pose of the robot
     * @param current_waypoint_index Current waypoint that the robot has just arrived at
     * @return true if task execution was successful, false otherwise
     */
    virtual bool processAtWaypoint(
        const geometry_msgs::msg::PoseStamped & current_pose,
        const int & current_waypoint_index) = 0;
};
} // namespace mp_core

#endif // MP_CORE__WAYPOINT_TASK_EXECUTOR_HPP_