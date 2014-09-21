cmake_minimum_required(VERSION 2.8.3)
project(vigir_pick_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  flor_grasp_msgs
  flor_ocs_msgs
  flor_control_msgs
  moveit_core
  moveit_msgs
  shape_msgs
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
        moveit_msgs
        shape_msgs
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
include/${PROJECT_NAME}/vigir_pick_controller.h
include/${PROJECT_NAME}/vigir_robotiq_pick_controller.h
)

SET(ROBOTIQ_SOURCES
src/vigir_pick_controller.cpp
src/vigir_robotiq_pick_controller.cpp
src/vigir_main_robotiq_pick_controller.cpp
)

add_executable(vigir_robotiq_pick_controller_node ${ROBOTIQ_SOURCES} ${HEADERS})
target_link_libraries(vigir_robotiq_pick_controller_node ${catkin_LIBRARIES} )
add_dependencies(vigir_robotiq_pick_controller_node ${catkin_EXPORTED_TARGETS})





