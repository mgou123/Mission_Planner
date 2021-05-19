#ifndef MP_CORE__PROGRESS_CHECKER_HPP_
#define MP_CORE__PROGRESS_CHECKER_HPP_

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mp_core
{
/**
 * @class mp_core::ProgressChecker
 * @brief This class defines the plugin interface used to check the
 * position of the robot to make sure that it is actually 
 * progressing towards a goal.
 */
class ProgressChecker
{
public:
    typedef std::shared_ptr<mp_core::ProgressChecker> Ptr;

    virtual ~ProgressChecker() {}

    /**
     * @brief Initialize parameters for ProgressChecker
     * @param parent Node pointer
     */
    virtual void initialize(
        const ros::NodeHandle & parent,
        const std::string & plugin_name) = 0;
    
    /**
     * @brief Checks if the robot has moved compared to previous pose
     * @param current_pose Current pose of the robot
     * @return True if progress is made
     */
    virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;

    /**
     * @brief Reset class state upon calling
     */
    virtual void reset() = 0;
};

} // namespace mp_core

#endif // MP_CORE__PROGRESS_CHECKER_HPP_