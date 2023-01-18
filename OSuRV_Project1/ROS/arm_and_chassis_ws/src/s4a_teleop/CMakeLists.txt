cmake_minimum_required(VERSION 2.8.3)

project(s4a_teleop)



find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	geometry_msgs
	moveit_servo
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

		moveit_servo

)


#############
## Install ##
#############

catkin_install_python(
	PROGRAMS 

		src/jog_teleop.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
