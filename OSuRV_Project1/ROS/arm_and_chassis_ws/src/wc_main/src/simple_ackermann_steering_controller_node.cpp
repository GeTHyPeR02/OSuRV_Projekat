
#include "simple_ackermann_steering_controller.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_ackermann_steering_controller");
	ros::NodeHandle nh;

	SimpleAckermannSteeringController chassis(nh);
	
	ros::MultiThreadedSpinner spinner(2); //TODO
	spinner.spin();
	
	//TODO Exit gracefully from here on Ctrl+C signal: 
	
	return 0;
}
