#! /bin/sh
cd /home/pi/Documents/projekat/OSuRV_Project/ROS/arm_and_chassis_ws
source devel/setup.bash
rostopic pub -1 /common/run_routine std_msgs/String 'data: "r1.moveitcmd"'