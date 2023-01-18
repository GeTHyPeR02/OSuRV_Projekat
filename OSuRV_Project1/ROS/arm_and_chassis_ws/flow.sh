#!/bin/bash

exit 1


######
# Build.
catkin_make -j1
source devel/setup.bash

######
# Run arm.

# T1
roslaunch s4a_main main.launch all_motors_sim:=false
# T2
tail --follow `roslaunch-logs`/robot_hardware_interface__joints.log
# T3
roslaunch common_teleop routines_teleop.launch
# T4
rostopic echo /tf_publish/beam00_base__to__beam2_hand_ee
# T5
# Joypad must have analog on.
roslaunch s4a_teleop servo_teleop.launch

# On Joypad:
# Jog.
# Select to go to servo mode.
# RT and RL to select routine.
# 1 to start routine.

######
# Run chassis.

# T1
roslaunch wc_main main.launch all_motors_sim:=false
# T2
roslaunch common_teleop routines_teleop.launch
# T3
roslaunch wc_teleop manual_teleop.launch

######
# Setup arm.
roslaunch s4a_main check_geom.launch
roslaunch moveit_setup_assistant setup_assistant.launch


######
# Helpers.

# Design routine.
rosrun moveit_commander moveit_commander_cmdline.py arm

# Change controller.
rosrun s4a_teleop change_controller.py servo



# Get pose:
rosrun tf tf_echo base_beam hand_ee_beam
rosrun moveit_commander moveit_commander_cmdline.py arm << EOF
current
EOF
rostopic echo -n 1 /joint_states
rostopic echo /s4a/traj_pos_ctrl/command
rostopic echo /s4a/jog_pos_ctrl


#Servo server.
rostopic echo /s4a/servo_pos_ctrl/command
rostopic echo /servo_server/delta_twist_cmds
rostopic echo /servo_server/status

######
# Without joypad:

# Servo
rostopic pub -r 25 -s /joypad_teleop/delta_twist_cmds geometry_msgs/TwistStamped "
header: auto
twist:
  linear:
    x: 0.0
    y: 0.0
    z: -1.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
"

rostopic pub -1 /s4a/motors_en std_msgs/Bool "
data: true
"


# Drifting
rosservice call /servo_server/change_drift_dimensions "
drift_x_translation: false
drift_y_translation: false
drift_z_translation: false
drift_x_rotation: true
drift_y_rotation: false
drift_z_rotation: true
transform_jog_frame_to_drift_frame:
  translation: {x: 0.0, y: 0.0, z: 0.0}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
"


# List controllers:
rosrun controller_manager controller_manager list
# Switch controllers:
rosservice call /controller_manager/switch_controller "
start_controllers: ['/s4a/traj_pos_ctrl']
stop_controllers: ['/s4a/jog_pos_ctrl', '/s4a/traj_pos_ctrl/arm', '/s4a/traj_pos_ctrl/hand', '/s4a/servo_pos_ctrl']
strictness: 1
start_asap: false
timeout: 0.0
"

######
# Tuning:
rosrun rqt_gui rqt_gui
# Plugins -> Configuration -> Dynamic Reconfigure

# View TF tree
rosrun tf view_frames
