#!/usr/bin/python

import rospy

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

import math

# FIXME for now is %
MAX_SPEED = 100 # [m/s]
MAX_STEERING = math.radians(30) # [rad]



class ManualTeleop:
	def __init__(self):
		
		self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
		self.cmd_pub = rospy.Publisher(
			'cmd',
			AckermannDriveStamped,
			queue_size = 1
		)
		
		rospy.loginfo('''

Manual Teleop:
	
	L stick:
		U	+axis	Go forward
		D	-axis	Go backward
		L	+axis	Steer left
		R	-axis	Steer right
		
		''')

	def on_joy(self, data):
		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'chassis'
		#TODO Limits ROC:
		# msg.drive.steering_angle_velocity
		# msg.drive.acceleration
		
		# Cross R/L | L stick R/L
		msg.drive.steering_angle = data.axes[0] * MAX_STEERING
		# Cross D/U | L stick D/U
		msg.drive.speed = data.axes[1] * MAX_SPEED
		
		self.cmd_pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('manual_teleop')
	controller = ManualTeleop()
	rospy.spin()
