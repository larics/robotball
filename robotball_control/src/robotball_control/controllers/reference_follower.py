#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import copy
import rospy

from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Vector3, Quaternion, Pose
from visualization_msgs.msg import Marker

from robotball_path.cfg import FollowerConfig

from pid import PID


class RefTracker(object):
    def __init__(self):
        # Variables.
        self.reference = Pose2D()
        self.odom = Odometry()
        self.odom_old = Odometry()

        rate = 10
        c = rospy.get_param('/position_pid')
        self.pid_x = PID(c['P'], c['I'], c['D'], 1.0 / rate)
        self.pid_y = PID(c['P'], c['I'], c['D'], 1.0 / rate)
        self.pid_limit = c['lim']

        # Publishers.
        self.vel_pub = rospy.Publisher('ref_vel', Twist, queue_size=1)

        # Dynamic reconfigure server.
        Server(FollowerConfig, self.reconf_cb)

        # Visaulization marker
        marker = Marker()
        marker.ns = rospy.get_namespace()
        marker.header.frame_id = rospy.get_namespace() + 'base_link'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.color = ColorRGBA(0, 1, 1, 1)
        marker.pose = Pose()
        marker.pose.position.z = 0.1776  # Sphero radius
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        self.marker_pub = rospy.Publisher('cmd_viz', Marker, queue_size=1)

        # Subscribers.
        rospy.Subscriber('pos_ref', Pose2D, self.ref_cb, queue_size=1)
        rospy.Subscriber('odom_estimated', Odometry, self.odom_cb, queue_size=1)

        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # CONTROL
            cmd_vel = self.compute()
            self.vel_pub.publish(cmd_vel)

            # VISUALIZATION
            angle = Quaternion(*quaternion_from_euler(0, 0, cmd_vel.linear.y))
            scale = Vector3(cmd_vel.linear.x, 0.08, 0.08)

            marker.header.stamp = rospy.get_rostime()
            marker.pose.orientation = angle
            marker.scale = scale
            self.marker_pub.publish(marker)

            r.sleep()

    def ref_cb(self, msg):
        self.reference = msg

    def odom_cb(self, msg):
        self.odom_old = copy.deepcopy(self.odom)
        self.odom = msg

    def reconf_cb(self, config, level):
        self.pid_x.kp = config['P']
        self.pid_x.ki = config['I']
        self.pid_x.kd = config['D']

        self.pid_y.kp = config['P']
        self.pid_y.ki = config['I']
        self.pid_y.kd = config['D']

        self.pid_limit = config['lim']

        return config

    def get_relative_motion(self):
        pos_new = self.odom.pose.pose.position
        pos_old = self.odom_old.pose.pose.position

        dist = math.sqrt((pos_new.x - pos_old.x) ** 2 + (pos_new.y - pos_old.y) ** 2)
        angle = math.atan2(pos_new.y - pos_old.y, pos_new.x - pos_old.x)

        return dist, angle

    def compute(self):
        cmd_vel = Twist()

        x = self.pid_x.compute(self.reference.x, self.odom.pose.pose.position.x)
        y = self.pid_y.compute(self.reference.y, self.odom.pose.pose.position.y)

        cmd_vel.linear.x = min(math.sqrt(x**2 + y**2), self.pid_limit)
        cmd_vel.linear.y = math.atan2(y, x)

        return cmd_vel


if __name__ == "__main__":
    rospy.init_node("ref_follower")

    try:
        node = RefTracker()
    except rospy.ROSInterruptException:
        pass
