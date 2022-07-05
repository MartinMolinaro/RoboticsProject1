cmake_minimum_required(VERSION 3.0.2)


project(ros1)
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs
	tf
	geometry_msgs
	sensor_msgs 
	nav_msgs
	message_generation
	dynamic_reconfigure
	)

generate_dynamic_reconfigure_options(
		cfg/integration.cfg
)

add_service_files(
		FILES
		reset_pos.srv
		reset_ticks.srv
		calibration.srv
)

add_message_files(
	FILES 
	WheelSpeed.msg
	)

generate_messages(
	DEPENDENCIES 
	nav_msgs
	geometry_msgs
	sensor_msgs
	std_msgs
	)

catkin_package(
	CATKIN_DEPENDS
	std_msgs
	message_runtime
	geometry_msgs
	sensor_msgs
	)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ComputeOdometry src/ComputeOdometry.cpp)
target_link_libraries(ComputeOdometry ${catkin_LIBRARIES})



#For custom message generated in the package
add_dependencies(ComputeOdometry ros1_generate_messages_cpp)

#For dynamic reconfigure
add_dependencies(ComputeOdometry ${PROJECT_NAME}_gencfg)


#For services
add_dependencies(ComputeOdometry ${catkin_EXPORTED_TARGETS})


