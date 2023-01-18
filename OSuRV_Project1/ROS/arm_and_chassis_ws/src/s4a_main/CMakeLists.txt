cmake_minimum_required(VERSION 2.8.3)

project(s4a_main)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	controller_manager
	srdfdom
)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp controller_manager srdfdom
	# DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	include
)

add_executable(
	robot_hardware_interface
	src/exec.cpp
	src/robot_hardware_interface.cpp
	src/robot_hardware_interface_node.cpp
)
target_link_libraries(
	robot_hardware_interface
	${catkin_LIBRARIES}
	urdfdom_model
)


#############
## Install ##
#############


# Install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
