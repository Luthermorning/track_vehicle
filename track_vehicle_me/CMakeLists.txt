cmake_minimum_required(VERSION 2.8.3)

project(track_vehicle_me)

find_package(catkin REQUIRED
	roscpp
	rospy
	std_msgs
	gazebo_msgs
	gazebo_plugins
	gazebo_ros
	gazebo_ros_control
	controller_manager
	joint_state_controller
	robot_state_publisher
	geometry_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(track_vehicle_teleop src/track_vehicle_teleop.cpp)
add_executable(turn src/turn.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(track_vehicle_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(turn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(track_vehicle_teleop
  ${catkin_LIBRARIES}
)

target_link_libraries(turn
  ${catkin_LIBRARIES}
)

