cmake_minimum_required(VERSION 2.8.3)

project(common_teleop)


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	geometry_msgs
	tf
	ackermann_msgs
	nav_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
		rospy
		roscpp
		geometry_msgs
		tf
		ackermann_msgs
		nav_msgs
	DEPENDS
		EIGEN3
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)
add_executable(
	tf_publish
	src/tf_publish.cpp
)
target_link_libraries(
	tf_publish
	${catkin_LIBRARIES}
)



#############
## Install ##
#############

catkin_install_python(
	PROGRAMS 
		scripts/change_controller.py
		src/mode_teleop.py
		src/routines_teleop.py
		src/routines_server.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
