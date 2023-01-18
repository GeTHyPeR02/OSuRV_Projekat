cmake_minimum_required(VERSION 2.8.3)

project(wc_teleop)


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	geometry_msgs
	ackermann_msgs
	nav_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
	# INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
		rospy
		roscpp
		geometry_msgs
)




#############
## Install ##
#############

catkin_install_python(
	PROGRAMS 
		src/manual_teleop.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
