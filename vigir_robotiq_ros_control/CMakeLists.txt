cmake_minimum_required(VERSION 2.8.3)
project(vigir_robotiq_ros_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  atlas_msgs
  flor_grasp_msgs
  flor_ocs_msgs
  flor_control_msgs
  moveit_core
  robotiq_s_model_control
  hardware_interface
  controller_manager
)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        trajectory_msgs
        atlas_msgs
        flor_grasp_msgs
        flor_ocs_msgs
        flor_control_msgs
        moveit_core
        robotiq_s_model_control
        hardware_interface
        controller_manager
)


include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

SET(HEADERS
include/${PROJECT_NAME}/vigir_robotiq_ros_controller.h
)

SET(SOURCE
src/vigir_robotiq_ros_controller.cpp
)

add_executable(vigir_robotiq_ros_controller_node ${SOURCE} ${HEADERS})
target_link_libraries(vigir_robotiq_ros_controller_node ${catkin_LIBRARIES} )
add_dependencies(vigir_robotiq_ros_controller_node ${catkin_EXPORTED_TARGETS})
