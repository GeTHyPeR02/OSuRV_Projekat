cmake_minimum_required(VERSION 2.8.3)

project(wc_main)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	ackermann_msgs
	nav_msgs
)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp ackermann_msgs nav_msgs
	# DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	../../../../SW/Driver/motor_ctrl/include/
)

add_executable(
	simple_ackermann_steering_controller
	src/simple_ackermann_steering_controller.cpp
	src/simple_ackermann_steering_controller_node.cpp
)
target_link_libraries(
	simple_ackermann_steering_controller
	${catkin_LIBRARIES}
)


#############
## Install ##
#############


# Install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
