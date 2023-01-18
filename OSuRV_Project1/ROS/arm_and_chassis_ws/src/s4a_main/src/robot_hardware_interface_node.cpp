
#include "robot_hardware_interface.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_hardware_interface");
	ros::NodeHandle nh;

	RobotHardwareInterface robot(nh);
	
	// Needed threads = 1 + number of controller_manager calls in launch.
	ros::MultiThreadedSpinner spinner(6);
	spinner.spin();
	
	//TODO Exit gracefully from here on Ctrl+C signal: 
	// wait serial master to finish transactions and close UART.
	
	return 0;
}
