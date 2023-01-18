#! /bin/sh
cd /home/pi/Documents/projekat/OSuRV_Project/SW/Driver/motor_ctrl
make
make start

cd /home/pi/Documents/projekat/OSuRV_Project/ROS/arm_and_chassis_ws
catkin_make -j1
source devel/setup.bash
roslaunch s4a_main main.launch all_motors_sim:=false

