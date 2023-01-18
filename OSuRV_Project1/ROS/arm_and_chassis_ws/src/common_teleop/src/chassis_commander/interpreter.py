#!/usr/bin/python

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class ChassisInfoLevel:
	FAIL = 1
	WARN = 2
	SUCCESS = 3
	INFO = 4
	DEBUG = 5

class ChassisCommandInterpreter:
	def __init__(self):
		self.cmd_pub = rospy.Publisher(
			'cmd',
			AckermannDriveStamped,
			queue_size = 1
		)
		
		self.odom_sub = rospy.Publisher(
			'odom',
			Odometry,
			self.on_odom
		)
		
		
	def get_active_group(self):
		return "chassis"
		
	def execute(self, line):
		sl = line.split()
		if sl[0] == "go":
			if len(sl) != 4:
				return (
					ChassisInfoLevel.FAIL,
					"Wrong number of params for go command"
				)
			else:
				#TODO Limits
				speed_percents = float(sl[1])
				steer_percents = float(sl[2])
				distance_m = float(sl[3])
				
				#TODO Send for 2 sec, later odom from distance_m
				for iter in range(2*25):
					msg = AckermannDriveStamped()
					msg.header.stamp = rospy.Time.now()
					msg.header.frame_id = 'chassis'
					
					# Cross R/L | L stick R/L
					msg.drive.steering_angle = speed_percents/100 * MAX_STEERING
					# Cross D/U | L stick D/U
					msg.drive.speed = steer_percents/100 * MAX_SPEED
					
					self.cmd_pub.publish(msg)
					
		else:
			return (
				ChassisInfoLevel.FAIL,
				"Non existing command \"{}\"".format(sl[0])
			)
	
	def on_odom(self, data):
		rospy.logwarn(data)
	
	
