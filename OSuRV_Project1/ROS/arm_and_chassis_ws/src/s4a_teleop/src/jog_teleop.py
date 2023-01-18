#!/usr/bin/python

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray
from urdf_parser_py.urdf import URDF

import subprocess
import math

if False:
	import os
	import sys
	sys.path.append(os.path.dirname(os.path.realpath(__file__)))
	from common.utils import show

N_JOINTS = 4 # Reat from limits.

def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v

class JogTeleop:
	def __init__(self):
		self.first_time = True
		
		self.pos_cmd = fill(0, N_JOINTS)
		robot = URDF.from_parameter_server()
		self.pos_min = []
		self.pos_max = []
		joint_names = []
		for name, joint in robot.joint_map.items():
			if name.startswith('joint'):
				joint_names.append(name)
				self.pos_min.append(joint.limit.lower)
				self.pos_max.append(joint.limit.upper)
		#print(joint_names)
		#print(self.pos_min)
		#print(self.pos_max)
		
		self.active_motor = 0
		self.motors_freewheel = False
		
		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
		self.pos_pub = rospy.Publisher(
			'jog_pos_cmd',
			Float64MultiArray,
			queue_size = 1
		)
		self.motors_freewheel_pub = rospy.Publisher(
			'motors_freewheel',
			Bool,
			queue_size = 1
		)
		
		rospy.loginfo('''

Jog Teleop:
	
	L stick:
		L	+axis	CW		Red
		R	-axis	CCW		Blue
		
	Buttons:
		LT	active motor ++
		LB	active motor --
		
		2	Toggle motors freewheel
		
		''')

	def on_joy(self, data):
		if self.first_time:
			self.first_time = False
			self.prev_buttons = data.buttons
			self.buttons_re = fill(False, len(data.buttons))
			return
		
		for i in range(len(self.buttons_re)):
			self.buttons_re[i] = not self.prev_buttons[i] and data.buttons[i]
		self.prev_buttons = data.buttons
		

		if self.buttons_re[4]: # LT
			# Move to next motor.
			self.active_motor += 1
			if self.active_motor == N_JOINTS:
				self.active_motor = 0
			rospy.loginfo('Active motor: {}'.format(self.active_motor))
		elif self.buttons_re[6]: # LB
			self.active_motor -= 1
			if self.active_motor == -1:
				self.active_motor = N_JOINTS-1
			rospy.loginfo('Active motor: {}'.format(self.active_motor))
		
		if self.buttons_re[1]: # 2
			self.motors_freewheel = not self.motors_freewheel
			if self.motors_freewheel:
				rospy.loginfo('Motors freewheel')
			else:
				rospy.loginfo('Motors powered')
			msg = Bool()
			msg.data = self.motors_freewheel
			self.motors_freewheel_pub.publish(msg)
		
		
		# Cross R/L | L stick R/L
		n = data.axes[0] # In range [-1, 1]
		id = self.active_motor
		if n < 0:
			self.pos_cmd[id] = -n*self.pos_min[id]
		else:
			self.pos_cmd[id] = n*self.pos_max[id]
		
		msg = Float64MultiArray()
		msg.data = self.pos_cmd
		self.pos_pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('jog_teleop')
	controller = JogTeleop()
	rospy.spin()
