#!/usr/bin/env python

import argparse
import subprocess

map = {
	'jog': ['/s4a/jog_pos_ctrl'],
	'traj': ['/s4a/traj_pos_ctrl/arm', '/s4a/traj_pos_ctrl/hand'],
	'servo': ['/s4a/servo_pos_ctrl'],
	'traj_servo': ['/s4a/traj_pos_ctrl/arm', '/s4a/traj_pos_ctrl/hand'],
}
def change_controller(controller_short_name, silent = True):
	
	if controller_short_name not in map:
		print('Non supported controller "{}"'.format(controller_short_name))
	else:
		controller_names = map[controller_short_name]
	
	cmd = 'rosservice call /controller_manager/switch_controller'.split()
	arg = '''
start_controllers: {controller_names}
stop_controllers: ['/s4a/jog_pos_ctrl', '/s4a/traj_pos_ctrl/arm', '/s4a/traj_pos_ctrl/hand', '/s4a/servo_pos_ctrl']
strictness: 1
start_asap: false
timeout: 0.0
'''.format(controller_names = controller_names)
	cmd.append(arg)
	
	print('Changing to controller: {}'.format(controller_short_name))
	if not silent:
		print(subprocess.list2cmdline(cmd))
	subprocess.run(cmd) # Does not work: eat new lines.
	
	#TODO Traj servo
	if controller_short_name in ['servo', 'traj_servo']:
		cmd = 'rosservice call /servo_server/change_drift_dimensions'.split()
		arg = '''
drift_x_translation: false
drift_y_translation: false
drift_z_translation: false
drift_x_rotation: true
drift_y_rotation: false
drift_z_rotation: true
transform_jog_frame_to_drift_frame:
  translation: {x: 0.0, y: 0.0, z: 0.0}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
'''
		cmd.append(arg)
		if not silent:
			print(subprocess.list2cmdline(cmd))
		subprocess.run(cmd) # Does not work: eat new lines.
	
	
if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description = __doc__
	)
	# Positional arguments
	l = list(map.keys())
	parser.add_argument(
		dest = 'controller_short_name',
		metavar = 'controller_short_name',
		type = str,
		nargs = 1,
		choices = l,
		help = 'Controller to switch to ({})'.format(', '.join(l))
	)
	parser.add_argument(
		'--silent',
		action="store_true",
		help = 'Print command'
	)
	args = parser.parse_args()
	
	change_controller(
		args.controller_short_name[0],
		silent = args.silent
	)
	
