#!/usr/bin/python

import rospy
import rospkg

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Bool

import os
import glob


def fill(fill_with, N):
	v = []
	for i in range(N):
		v.append(fill_with)
	return v
	

class RoutinesTeleop:
	def __init__(self):
		self.first_time = True
		
		rospack = rospkg.RosPack()
		pkg_dir = rospack.get_path('common_teleop')
		self.routines_dir = os.path.join(pkg_dir, 'routines/')
		
		self.active_routine = ''
		
		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
		
		self.run_routine_pub = rospy.Publisher(
			'run_routine',
			String,
			queue_size = 1
		)

		rospy.loginfo('''

Routines Teleop:
	
	Buttons:
		RT	active routine ++
		RB	active routine --
		
		1	Run routine
		
		
		''')
	
	'''
		without prefix "_" and with ".moveitcmd" as extension
	'''
	def get_list_of_routines(self):
		r = []
		for ext in ['*.moveitcmd', '*.chassiscmd']:
			p = os.path.join(self.routines_dir, ext)
			g = glob.glob(p)
			for gg in g:
				b = os.path.basename(gg)
				if not b.startswith('_'):
					r.append(b)
		return r
	
	def set_active_routine(self, i_dir = 0):
		r = self.get_list_of_routines()
		if self.active_routine in r:
			i = r.index(self.active_routine)
			i += i_dir
			if i == len(r):
				i = 0
			elif i == -1:
				i = len(r)-1
			self.active_routine = r[i]
		else:
			if len(r) > 0:
				self.active_routine = r[0]
			else:
				self.active_routine = ''
		
	def on_joy(self, data):
		if self.first_time:
			self.first_time = False
			self.prev_buttons = data.buttons
			self.buttons_re = fill(False, len(data.buttons))
			
			self.set_active_routine()
			rospy.loginfo('Active routine: {}'.format(self.active_routine))
			
			return
		
		for i in range(len(self.buttons_re)):
			self.buttons_re[i] = not self.prev_buttons[i] and data.buttons[i]
		self.prev_buttons = data.buttons

		if self.buttons_re[5]: # RT
			self.set_active_routine(1)
			rospy.loginfo('Active routine: {}'.format(self.active_routine))
		elif self.buttons_re[7]: # RB
			self.set_active_routine(-1)
			rospy.loginfo('Active routine: {}'.format(self.active_routine))
			
		if self.buttons_re[0]: # 1
			msg = String()
			self.set_active_routine()
			rospy.logdebug('Running routine: {}'.format(self.active_routine))
			msg.data = self.active_routine
			self.run_routine_pub.publish(msg)
			
			

if __name__ == '__main__':
	rospy.init_node('routines_teleop')
	controller = RoutinesTeleop()
	rospy.spin()
