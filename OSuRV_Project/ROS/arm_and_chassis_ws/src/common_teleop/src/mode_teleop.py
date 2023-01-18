#!/usr/bin/python

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import subprocess

modes = ['chassis', 'jog', 'servo']

def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v

class ModeTeleop:
	def __init__(self):
		self.first_time = True
		
		self.mode = modes[0]
		self.s4a_motors_en = True
		self.wc_motors_en = True
		
		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
		
		self.s4a_motors_en_pub = rospy.Publisher(
			's4a/motors_en',
			Bool,
			queue_size = 1
		)
		self.wc_motors_en_pub = rospy.Publisher(
			'wc/motors_en',
			Bool,
			queue_size = 1
		)
		
		rospy.loginfo('''

Mode Teleop:
	
	Buttons:
		Select: change mode
		
		Start: Start/Stop motors
		
		''')

	def publish_motors_en(self):
		msg = Bool()
		msg.data = self.s4a_motors_en
		self.s4a_motors_en_pub.publish(msg)
		msg.data = self.wc_motors_en
		self.wc_motors_en_pub.publish(msg)
	

	def on_joy(self, data):
		if self.first_time:
			self.first_time = False
			self.prev_buttons = data.buttons
			self.buttons_re = fill(False, len(data.buttons))
			return
		
		for i in range(len(self.buttons_re)):
			self.buttons_re[i] = not self.prev_buttons[i] and data.buttons[i]
		self.prev_buttons = data.buttons
		
		
		if self.buttons_re[8]: # Select
			i = modes.index(self.mode) + 1
			assert(i <= len(modes))
			if i == len(modes):
				i = 0
			self.mode = modes[i]
			rospy.loginfo('Current mode: {}'.format(self.mode))
			if self.mode == 'chassis':
				self.s4a_motors_en = False
				self.wc_motors_en = True
				self.publish_motors_en()
			else:
				self.s4a_motors_en = True
				self.wc_motors_en = False
				self.publish_motors_en()
				if self.mode == 'jog':
					ctrl = 'jog'
				elif self.mode == 'servo':
					ctrl = 'servo'
					#ctrl = 'traj_servo'
				cmd = 'rosrun common_teleop change_controller.py --silent'.split()
				cmd.append(ctrl)
				subprocess.run(cmd)
			
			
		if self.buttons_re[9]: # Start
			if self.s4a_motors_en or self.wc_motors_en:
				rospy.loginfo('Motors disabled')
				self.s4a_motors_en = False
				self.wc_motors_en = False
			else:
				rospy.loginfo('Motors enabled')
				if self.mode == 'chassis':
					self.s4a_motors_en = False
					self.wc_motors_en = True
				else:
					self.s4a_motors_en = True
					self.wc_motors_en = False
			self.publish_motors_en()
		

if __name__ == '__main__':
	rospy.init_node('mode_teleop')
	controller = ModeTeleop()
	rospy.spin()
