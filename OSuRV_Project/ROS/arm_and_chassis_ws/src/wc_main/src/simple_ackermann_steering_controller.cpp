
#include "simple_ackermann_steering_controller.hpp"

// Config.
#define LOOP_HZ 25

#include "motor_ctrl.h"


// Includes for driver.
#include <string.h> // strerror()
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <sys/ioctl.h> // ioctl()


#include <algorithm>
using namespace std;

#include <angles/angles.h>


#define DEBUG(x) do{ ROS_DEBUG_STREAM(#x << " = " << x); }while(0)

template<typename T>
static T sym_clamp(T x, T limit) {
	return clamp(x, -limit, limit);
}

SimpleAckermannSteeringController::SimpleAckermannSteeringController(ros::NodeHandle& nh) :
	nh(nh),
	all_motors_sim(false),
	drv_fd(-1),
	motors_en(true)
{
	
	
	ros::param::param("~all_motors_sim", all_motors_sim, false);
	ROS_INFO(
		"all_motors_sim = %s",
		all_motors_sim ? "true" : "false"
	);
	
	
	if(!all_motors_sim){
		// Open driver.
		drv_fd = open(DEV_FN, O_RDWR);
		if(drv_fd < 0){
			ROS_WARN(
				"\"%s\" not opened! drv_fd = %d -> %s",
				DEV_FN, drv_fd, strerror(-drv_fd)
			);
		}
		
		motor_ctrl__ioctl_arg_moduo_t ia;
		ia.ch = 0;
		ia.moduo = 20; // 5kHz
		int r = ioctl(
			drv_fd,
			IOCTL_MOTOR_CLTR_SET_MODUO,
			*(unsigned long*)&ia
		);
		if(r){
			ROS_WARN("ioctl went wrong!");
		}
	}
	
	motors_en_sub = nh.subscribe(
		"motors_en",
		1,
		&SimpleAckermannSteeringController::motors_en_cb,
		this
	);
	
	cmd_sub = nh.subscribe(
		"cmd",
		1,
		&SimpleAckermannSteeringController::cmd_cb,
		this
	);
	
	
	ros::Duration timer_period = ros::Duration(1.0/LOOP_HZ);
	non_realtime_loop_timer = nh.createTimer(
		timer_period,
		&SimpleAckermannSteeringController::publish_odom,
		this
	);
	
	odom_pub  = nh.advertise<nav_msgs::Odometry>(
		"odom",
		1
	);
	
	odom_msg.header.frame_id = "bldc_enc";
	last = ros::Time::now();
}

SimpleAckermannSteeringController::~SimpleAckermannSteeringController() {
	if(!all_motors_sim){
		// Close driver.
		ROS_INFO("closing drv_fd");
		close(drv_fd);
	}
}

void SimpleAckermannSteeringController::seek() {
	const int o = 0;
	int r = lseek(drv_fd, SEEK_SET, o);
	if(r != o){
		if(r < 0){
			ROS_WARN(
				"lseek failed! errno = %d -> %s",
				errno, strerror(errno)
			);
		}else{
			ROS_WARN(
				"lseek failed! offset is %d but expected %d",
				r, o
			);
		}
	}
}

void SimpleAckermannSteeringController::publish_odom(const ros::TimerEvent& e) {
	ros::Time now = ros::Time::now();
	
	if(!all_motors_sim){
		seek();
		
		motor_ctrl__read_arg_fb_t ra;
		int r = read(drv_fd, (char*)&ra, sizeof(ra));
		if(r != sizeof(ra)){
			ROS_WARN("read went wrong!");
		}
		
		int64_t pulse_cnt_fb = ra.pulse_cnt_fb[0];
		ROS_DEBUG("pulse_cnt_fb = %lld\n", (long long)pulse_cnt_fb);
		traversed_path = pulse_cnt_fb * 0.1; //TODO Calib this constant.
	}else{
		traversed_path += speed * (now - last).toSec();
		last = now;
	}
	
	odom_msg.header.seq++;
	odom_msg.header.stamp = now;
	odom_msg.pose.pose.position.x = traversed_path;
	
	odom_pub.publish(odom_msg);
	
}

void SimpleAckermannSteeringController::motors_en_cb(
	const std_msgs::Bool::ConstPtr& msg
) {
	motors_en = msg->data;
	ROS_INFO(
		"Motors %s", motors_en ? "enabled" : "disabled"
	);
}


void SimpleAckermannSteeringController::cmd_cb(
	const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg
) {
	//TODO Limits roc:
	// steering_angle_velocity
	// acceleration
	speed = msg->drive.speed;
	float steering_angle = msg->drive.steering_angle;
	
#if 0
		ROS_DEBUG("speed = %f", speed);
		ROS_DEBUG("steering_angle = %f", steering_angle);
#endif
	
	if(!all_motors_sim && motors_en){
		int16_t duty[2];
		
		// |speed| = [0, 100] -> threshold = [10, 0].
		// it will be <<1 in write() so then will be [20, 0].
		// duty will also calculate in direction.
		uint8_t abs_speed;
		uint16_t threshold;
		if(speed > 0){
			abs_speed = speed;
			threshold = (100-abs_speed)/10;
			duty[0] = threshold;
		}else{
			abs_speed = -speed;
			threshold = (100-abs_speed)/10;
			duty[0] = -threshold;
		}
		
		float sa = -steering_angle; // + is left.
		// -pi/2=2.5%=25 +pi/2=12.5%=125
		duty[1] = (sa + M_PI_2)*(100/M_PI) + 25;
		
		seek();
		
		int r = write(drv_fd, (char*)&duty, sizeof(duty));
		if(r != sizeof(duty)){
			ROS_WARN("write went wrong!");
		}
	}
}
