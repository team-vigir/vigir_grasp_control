cmake_minimum_required(VERSION 2.8.3)
project(vigir_grasp_controllers_old)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  atlas_msgs
  control_msgs
  flor_control_msgs
  flor_grasp_msgs
  flor_ocs_msgs
  geometric_shapes
  handle_msgs
  moveit_core
  moveit_msgs
  osrf_msgs
  robotiq_s_model_control
  roscpp
  tf
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib atlas_msgs control_msgs flor_control_msgs flor_grasp_msgs flor_ocs_msgs geometric_shapes handle_msgs moveit_core moveit_msgs osrf_msgs robotiq_s_model_control roscpp tf trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_grasp_controllers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

SET(HEADERS
include/${PROJECT_NAME}/vigir_grasp_controller.h
include/${PROJECT_NAME}/vigir_sandia_grasp_controller.h
include/${PROJECT_NAME}/vigir_irobot_grasp_controller.h
include/${PROJECT_NAME}/vigir_robotiq_grasp_controller.h
include/${PROJECT_NAME}/vigir_hook_grasp_controller.h
)

SET(SANDIA_SOURCES
src/vigir_grasp_controller.cpp
src/vigir_sandia_grasp_controller.cpp
src/main_sandia_grasp_controller.cpp
)

SET(IROBOT_SOURCES
src/vigir_grasp_controller.cpp
src/vigir_irobot_grasp_controller.cpp
src/main_irobot_grasp_controller.cpp
)

SET(ROBOTIQ_SOURCES
src/vigir_grasp_controller.cpp
src/vigir_robotiq_grasp_controller.cpp
src/main_robotiq_grasp_controller.cpp
)

SET(HOOK_SOURCES
src/vigir_grasp_controller.cpp
src/vigir_hook_grasp_controller.cpp
src/main_hook_grasp_controller.cpp
)

add_executable(sandia_grasp_controller_old_node ${SANDIA_SOURCES} ${HEADERS})
add_executable(sandia_joint_states_old_node src/vigir_sandia_joint_states)
add_executable(irobot_grasp_controller_old_node ${IROBOT_SOURCES} ${HEADERS})
add_executable(irobot_joint_states_old_node src/vigir_irobot_joint_states)
add_executable(robotiq_grasp_controller_old_node ${ROBOTIQ_SOURCES} ${HEADERS})
add_executable(robotiq_joint_states_old_node src/vigir_robotiq_joint_states)
add_executable(hook_grasp_controller_old_node ${HOOK_SOURCES} ${HEADERS})

target_link_libraries(sandia_grasp_controller_old_node  ${catkin_LIBRARIES})
target_link_libraries(sandia_joint_states_old_node      ${catkin_LIBRARIES})
target_link_libraries(irobot_grasp_controller_old_node  ${catkin_LIBRARIES})
target_link_libraries(irobot_joint_states_old_node      ${catkin_LIBRARIES})
target_link_libraries(robotiq_grasp_controller_old_node ${catkin_LIBRARIES})
target_link_libraries(robotiq_joint_states_old_node     ${catkin_LIBRARIES})
target_link_libraries(hook_grasp_controller_old_node    ${catkin_LIBRARIES})


add_dependencies(sandia_grasp_controller_old_node  ${catkin_EXPORTED_TARGETS})
add_dependencies(sandia_joint_states_old_node      ${catkin_EXPORTED_TARGETS})
add_dependencies(irobot_grasp_controller_old_node  ${catkin_EXPORTED_TARGETS})
add_dependencies(irobot_joint_states_old_node      ${catkin_EXPORTED_TARGETS})
add_dependencies(robotiq_grasp_controller_old_node ${catkin_EXPORTED_TARGETS})
add_dependencies(robotiq_joint_states_old_node     ${catkin_EXPORTED_TARGETS})
add_dependencies(hook_grasp_controller_old_node    ${catkin_EXPORTED_TARGETS})


install( TARGETS
    sandia_grasp_controller_old_node
    sandia_joint_states_old_node
    irobot_grasp_controller_old_node
    irobot_joint_states_old_node
    robotiq_grasp_controller_old_node
    robotiq_joint_states_old_node
    hook_grasp_controller_old_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install ( DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
