#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FakeRobot(object):
	def __init__(self):

		# Publishers
		self.odom_pub = rospy.Publisher('odom_estimated', Odometry, queue_size=1)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		# Subscribers
		rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)
		rospy.Subscriber('ref_vel', Twist, self.vel_cb, queue_size=1)

		rospy.spin()

	def odom_cb(self, msg):
		self.odom_pub.publish(msg)

	def vel_cb(self, msg):
		mag = msg.linear.x
		dir = msg.linear.y

		cmd_vel = Twist()
		cmd_vel.linear.x = mag * math.cos(dir)
		cmd_vel.linear.y = mag * math.sin(dir)

		self.vel_pub.publish(cmd_vel)


if __name__ == '__main__':
	rospy.init_node('fake_robot')

	try:
		node = FakeRobot()
	except rospy.ROSInterruptException:
		pass