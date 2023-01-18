
#pragma once

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <memory>
#include <fstream>




class RobotHardwareInterface : public hardware_interface::RobotHW {
public:
	RobotHardwareInterface(ros::NodeHandle& nh);
	~RobotHardwareInterface();
	void update(const ros::TimerEvent& e);
	void read(ros::Duration elapsed_time);
	void write(ros::Duration elapsed_time);
	
	
public:
	struct State {
		std::vector<double> pos_cmd; // [rad]
		std::vector<double> eff_fb; // [%]
		std::vector<double> pos_fb; // [rad]
		std::vector<double> vel_fb; // [rad/s]
		std::vector<double> acc; // [rad/s^2]
		
		State(){}
		State(size_t n_joints);
	};
	
protected:
	bool log_joint_states;
	std::ofstream joints_log;
	bool first_print;
	void print_array(
		std::ostream& os,
		const char* name,
		const std::vector<double>& curr,
		const std::vector<double>& prev,
		double scale = 1.0
	);
	void log_joint_states_fun(const ros::Time& t);
	
	bool all_motors_sim;
	
	int drv_fd;
	bool offset_channels;
	void seek();
	
	void readJointNames();
	size_t n_joints;
	std::vector<std::string> joint_names;
	State curr;
	State prev;
	
	std::vector<uint16_t> tmp_duty; // [permilles]
	
	hardware_interface::JointStateInterface fb_if;
	hardware_interface::PositionJointInterface cmd_if;
	
	ros::NodeHandle nh;
	ros::Timer non_realtime_loop_timer;
	controller_manager::ControllerManager* ctrl_manager;

	ros::Subscriber motors_en_sub;
	void motors_en_cb(const std_msgs::Bool::ConstPtr& msg);
	volatile bool motors_en;
	
	ros::Publisher acc_pub;
	std_msgs::Float64MultiArray acc_msg;
};
