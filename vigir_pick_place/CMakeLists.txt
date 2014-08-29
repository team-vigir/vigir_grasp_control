cmake_minimum_required(VERSION 2.8.3)
project(vigir_pick_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  flor_grasp_msgs
  flor_ocs_msgs
  flor_control_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  hardware_interface
  controller_manager
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        trajectory_msgs
        flor_grasp_msgs
        flor_ocs_msgs
        flor_control_msgs
        moveit_core
        moveit_ros_planning_interface
        interactive_markers
        hardware_interface
        controller_manager
)

find_package(Eigen REQUIRED)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

SET(HEADERS
include/${PROJECT_NAME}/vigir_pick_place.h
)

SET(SOURCE
src/vigir_pick_place.cpp
)

add_executable(vigir_pick_place_node ${SOURCE} ${HEADERS})
target_link_libraries(vigir_pick_place_node ${catkin_LIBRARIES} )
add_dependencies(vigir_pick_place_node ${catkin_EXPORTED_TARGETS})
