#!/usr/bin/python

import rospy
import rospkg

from std_msgs.msg import String

from moveit_commander import (
	MoveGroupCommandInterpreter,
	MoveGroupInfoLevel,
)

from chassis_commander import (
	ChassisCommandInterpreter,
	ChassisInfoLevel,
)

import os
import subprocess
import math


# FIXME for now is %
MAX_SPEED = 100 # [m/s]
MAX_STEERING = math.radians(30) # [rad]


class bcolors:
	HEADER = "\033[95m"
	OKBLUE = "\033[94m"
	OKGREEN = "\033[92m"
	WARNING = "\033[93m"
	FAIL = "\033[91m"
	ENDC = "\033[0m"
	
def print_message(level, msg):
	if level in [MoveGroupInfoLevel.FAIL, ChassisInfoLevel.FAIL]:
		print(bcolors.FAIL + msg + bcolors.ENDC)
	elif level in [MoveGroupInfoLevel.WARN, ChassisInfoLevel.WARN]:
		print(bcolors.WARNING + msg + bcolors.ENDC)
	elif level in [MoveGroupInfoLevel.SUCCESS, ChassisInfoLevel.SUCCESS]:
		print(bcolors.OKGREEN + msg + bcolors.ENDC)
	elif level in [MoveGroupInfoLevel.DEBUG, ChassisInfoLevel.DEBUG]:
		print(bcolors.OKBLUE + msg + bcolors.ENDC)
	else:
		print(msg)

def get_ext(fn):
	return os.path.splitext(fn)[1]

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('common_teleop')
routines_dir = os.path.join(pkg_dir, 'routines/')

class RoutineExecutor:
	def __init__(self, routine_basename):
		self.routine_basename = routine_basename
		#TODO Warn on non-existing file.
		self.ext = get_ext(routine_basename)
		if self.ext == '.chassiscmd':
			self.interp = ChassisCommandInterpreter()
		else:
			self.interp = MoveGroupCommandInterpreter()
	def run(self):
		self.include(self.routine_basename)
	
	def include(self, routine_basename):
		fn = os.path.join(routines_dir, routine_basename)
		with open(fn) as f:
			for line in f.readlines():
				if line.strip() == '':
					# Skip empty line.
					continue
				elif line.lstrip().startswith('#'):
					# Skip comment.
					continue
				elif line.lstrip().startswith('%include'):
					# Include recusively routine.
					rb2 = line.strip().split()[1]
					ext2 = get_ext(rb2)
					if ext2 != self.ext:
						print_message(
							1,
							"{rb} of type {ext} cannot include"\
								" {rb2} of type {ext2}".format(
								rb = routine_basename,
								ext = self.ext,
								rb2 = rb2,
								ext2 = ext2
							)
						)
					else:
						self.include(rb2)
				elif line.lstrip().startswith('%run'):
					# Run recusively routine. Will not save variables.
					rb2 = line.strip().split()[1]
					RoutineExecutor(rb2).run()
				else:
					# Input.
					name = ''
					ag = self.interp.get_active_group()
					if ag != None:
						name = ag.get_name()
					print(
						bcolors.OKBLUE + name + '> ' + bcolors.ENDC + line,
						end = ''
					)
					
					# Execute.
					(level, msg) = self.interp.execute(line)
					
					# Output.
					#TODO Print file and line.
					print_message(level, msg)
	

class RoutinesServer:
	def __init__(self):
		self.run_routine_sub = rospy.Subscriber(
			"run_routine",
			String,
			self.on_run_routine
		)


	
	def go_routine_mode_and_exec(self, routine_basename):
		rospy.loginfo('Running routine: {}'.format(routine_basename))
		
		# Change mode.
		def change_mode(mode):
			cmd = 'rosrun common_teleop change_controller.py --silent'.split()
			cmd.append(mode)
			subprocess.run(cmd)
		
		change_mode('traj')
		
		RoutineExecutor(routine_basename).run()

		# Return back.
		#TODO Need to read which controller was active.
		change_mode('servo')


	def on_run_routine(self, msg):
		routine_basename = msg.data
		self.go_routine_mode_and_exec(routine_basename)
		
		

if __name__ == '__main__':
	rospy.init_node('routines_server')
	controller = RoutinesServer()
	rospy.spin()
