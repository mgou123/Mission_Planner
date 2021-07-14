cmake_minimum_required(VERSION 3.0.2)
project(mp_behavior_tree)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  behaviortree_cpp_v3
  tf2_ros
  tf2
  tf2_geometry_msgs
  nav_utils
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    ${PROJECT_NAME}
    ${plugin_libs}
  CATKIN_DEPENDS 
    roscpp
    actionlib
    actionlib_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    behaviortree_cpp_v3
    nav_utils
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/behavior_tree_engine.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(mp_update_pose_bt_node SHARED plugins/action/update_pose.cpp)
list(APPEND plugin_libs mp_update_pose_bt_node)

add_library(mp_convert_odom_to_pose_bt_node SHARED plugins/action/convert_odom_to_pose.cpp)
list(APPEND plugin_libs mp_convert_odom_to_pose_bt_node)

add_library(mp_controller_service_bt_node SHARED plugins/action/controller_service.cpp)
list(APPEND plugin_libs mp_controller_service_bt_node)

add_library(mp_set_relative_goal_bt_node SHARED plugins/action/set_relative_goal.cpp)
list(APPEND plugin_libs mp_set_relative_goal_bt_node)

add_library(mp_get_yaw_between_poses_bt_node SHARED plugins/action/get_yaw_between_poses.cpp)
list(APPEND plugin_libs mp_get_yaw_between_poses_bt_node)

add_library(mp_get_distance_between_poses_bt_node SHARED plugins/action/get_distance_between_poses.cpp)
list(APPEND plugin_libs mp_get_distance_between_poses_bt_node)

add_library(mp_split_pose_components_bt_node SHARED plugins/action/split_pose_components.cpp)
list(APPEND plugin_libs mp_split_pose_components_bt_node)

add_library(mp_is_time_expired_condition_bt_node SHARED plugins/condition/is_time_expired_condition.cpp)
list(APPEND plugin_libs mp_is_time_expired_condition_bt_node)

add_library(mp_is_distance_traveled_condition_bt_node SHARED plugins/condition/is_distance_traveled_condition.cpp)
list(APPEND plugin_libs mp_is_distance_traveled_condition_bt_node)

add_library(mp_is_goal_reached_condition_bt_node SHARED plugins/condition/is_goal_reached_condition.cpp)
list(APPEND plugin_libs mp_is_goal_reached_condition_bt_node)

add_library(mp_is_goal_updated_condition_bt_node SHARED plugins/condition/is_goal_updated_condition.cpp)
list(APPEND plugin_libs mp_is_goal_updated_condition_bt_node)

add_library(mp_is_pose_updated_condition_bt_node SHARED plugins/condition/is_pose_updated_condition.cpp)
list(APPEND plugin_libs mp_is_pose_updated_condition_bt_node)

add_library(mp_dynamic_timeout_sequence_bt_node SHARED plugins/control/dynamic_timeout_sequence.cpp)
list(APPEND plugin_libs mp_dynamic_timeout_sequence_bt_node)

add_library(mp_dynamic_timeout_bt_node SHARED plugins/decorator/dynamic_timeout.cpp)
list(APPEND plugin_libs mp_dynamic_timeout_bt_node)

foreach(bt_plugin ${plugin_libs})
  add_dependencies(${bt_plugin} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${bt_plugin}
    ${catkin_LIBRARIES}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${PROJECT_NAME}
  ${plugin_libs}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mp_behavior_tree_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mp_behavior_tree.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
