#! /bin/sh
cd /home/pi/Documents/projekat/OSuRV_Project/ROS/arm_and_chassis_ws
source devel/setup.bash
rostopic pub -1 /common/run_routine std_msgs/String 'data: "r1.moveitcmd"'

ssh pi@10.1.207.146 'cd /home/pi/Pictures/OSuRV_Project/ROS/arm_and_chassis_ws; source devel/setup.bash; rostopic pub -1 /common/run_routine std_msgs/String "data: \"r2.moveitcmd\"" '

rostopic pub -1 /common/run_routine std_msgs/String 'data: "r12.moveitcmd"'

ssh pi@10.1.207.146 'cd /home/pi/Pictures/OSuRV_Project/ROS/arm_and_chassis_ws; source devel/setup.bash; rostopic pub -1 /common/run_routine std_msgs/String "data: \"r22.moveitcmd\"" '

