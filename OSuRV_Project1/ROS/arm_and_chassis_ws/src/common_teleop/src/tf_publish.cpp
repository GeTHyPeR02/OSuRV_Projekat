
#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "tf_publish", ros::init_options::AnonymousName);

	// Allow 2 or 3 command line arguments
	if (argc < 3 || argc > 4){
		printf("Usage: tf_publish source_frame target_frame [echo_rate]\n\n");
		printf("This will publish the transform from the coordinate frame of the source_frame\n");
		printf("to the coordinate frame of the target_frame as TwistStamped. \n");
		printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
		printf("Default echo rate is 1 if echo_rate is not given.\n");
		return -1;
	}

	ros::NodeHandle nh("~");

	double rate_hz;
	if (argc == 4){
		// read rate from command line
		rate_hz = atof(argv[3]);
	}else{
		// read rate parameter
		nh.param("rate", rate_hz, 1.0);
	}
	if(rate_hz <= 0.0){
		std::cerr << "Echo rate must be > 0.0\n";
		return -1;
	}
	ros::Rate rate(rate_hz);

	tf::TransformListener tf;

	std::string source_frameid = std::string(argv[1]);
	std::string target_frameid = std::string(argv[2]);

	// Wait for up to one second for the first transforms to become avaiable. 
	tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

	std::string topic_name = 
		"/tf_publish/" + source_frameid + "__to__" + target_frameid;
	ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>(
		topic_name,
		1
	);

	while(nh.ok()){
		try{
			tf::StampedTransform echo_transform;
			// 'beam00_base' and 'beam2_hand_ee' are not in the same tree.
			tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
			double yaw, pitch, roll;
			echo_transform.getBasis().getRPY(roll, pitch, yaw);
			tf::Quaternion q = echo_transform.getRotation();
			tf::Vector3 v = echo_transform.getOrigin();
			//print transform
			geometry_msgs::TwistStamped msg;
			msg.twist.linear.x = v.getX();
			msg.twist.linear.y = v.getY();
			msg.twist.linear.z = v.getZ();
			msg.twist.angular.x = roll;
			msg.twist.angular.y = pitch;
			msg.twist.angular.z = yaw;
			//TODO msg.header.seq;
			msg.header.stamp = echo_transform.stamp_;
			pub.publish(msg);
		}catch(tf::TransformException& ex){
//TODO Better.
#if 0
			std::cout << "Failure at "<< ros::Time::now() << std::endl;
			std::cout << "Exception thrown:" << ex.what()<< std::endl;
			std::cout << "The current list of frames is:" <<std::endl;
			std::cout << tf.allFramesAsString() << std::endl;
#endif
		}
		rate.sleep();
	}

	return 0;
}
