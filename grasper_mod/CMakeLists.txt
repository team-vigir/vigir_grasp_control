# Team VIGIR DRC OpenRAVE grasper plugin
###########################################
cmake_minimum_required (VERSION 2.8.3)
project (grasper_mod)
find_package(OpenRAVE REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
	roslib 
	roscpp 
	std_msgs
	moveit_msgs
	hullify
	rave_to_moveit
)

#find_package(qhull REQUIRED)
INCLUDE (CheckIncludeFiles)

# Add this CMakeList to the catkin build setup
catkin_package(
	INCLUDE_DIRS
	LIBRARIES grasper_mod
	CATKIN_DEPENDS roslib roscpp std_msgs moveit_msgs hullify
	DEPENDS OpenRAVE
)

## check qhull library
check_library_exists(qhull qh_new_qhull "" QHULL_FOUND_LIB)
CHECK_INCLUDE_FILES(qhull/qhull_a.h HAVE_QHULL_H)
if( QHULL_FOUND_LIB AND HAVE_QHULL_H )
	set(QHULL_FOUND 1)
	set(QHULL_INCLUDE_DIR 1)
	message(STATUS "Found qhull!")
else()
	# have to compile from sources
	message(STATUS "compiling local qhull library")
	add_subdirectory(3rdparty/qhull)
	set(QHULL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty")
	set(QHULL_FOUND 1)
endif()

include_directories(${OpenRAVE_INCLUDE_DIRS} mods .)
include_directories(${hullify_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} `rospack find grasper_mod`)

if( QHULL_FOUND AND QHULL_INCLUDE_DIR )
	message(STATUS "including QHULL include dirs")
	include_directories("${QHULL_INCLUDE_DIR}")
else()
	message(STATUS "NOT including QHULL")
endif()

## Build the modified grasper plugin
#message(STATUS "Include Directories: ${INCLUDE_DIRECTORIES}")
add_library(grasper_mod SHARED grasper.cpp graspermodule.cpp grasperplanner.cpp  plugindefs.h mods/ros_interface.cpp)
add_dependencies(grasper_mod DEPENDS moveit_msgs_generate_messages_cpp osu_grasp_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
set_target_properties(grasper_mod PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
if( QHULL_FOUND )
	target_link_libraries(grasper_mod ${OpenRAVE_LIBRARIES} qhull ${catkin_LIBRARIES} ${hullify_LIBRARIES})
	add_definitions(-DQHULL_FOUND)
else()
	target_link_libraries(grasper_mod ${OpenRAVE_LIBRARIES} ${catkin_LIBRARIES})
endif()


## Place the result in the rave_to_moveit directory for immediate use
##	Also place it in the build folder as a backup
execute_process(COMMAND rospack find rave_to_moveit OUTPUT_VARIABLE ROS_RAVE_DEST_DIR_RAW)
string(STRIP ${ROS_RAVE_DEST_DIR_RAW} ROS_RAVE_DEST_DIR)
message(STATUS "Rave_to_moveit dest dir: ${ROS_RAVE_DEST_DIR}/plugins")
#install(TARGETS grasper_mod DESTINATION ./build COMPONENT ${COMPONENT_PREFIX}plugin-grasper_mod)
install(TARGETS grasper_mod ARCHIVE DESTINATION ${ROS_RAVE_DEST_DIR}/plugins LIBRARY DESTINATION ${ROS_RAVE_DEST_DIR}/plugins CONFIGURATIONS Release Debug COMPONENT openrave_plugin PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE GROUP_WRITE)
