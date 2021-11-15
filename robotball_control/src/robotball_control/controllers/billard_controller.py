#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import copy
import random
import collections
import numpy as np
import rospy

from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Vector3, Quaternion, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray

from robotball_msgs.msg import IMU as myIMU

from robotball_control.cfg import BillardConfig

from util import Vector2, wrap_pi_pi


class BilliardController(object):
    def __init__(self):
        # Get parameters.
        default = rospy.get_param('/billiard_params')
        limits = default['limits']
        self.config = default['config']

        # Variables.
        self.me = rospy.get_namespace().strip('/')
        self.poses = {name: Pose2D() for name in default['robots']}

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.config['set_speed']
        cmd_vel_msg.linear.y = random.uniform(-math.pi, math.pi)

        bounced_x = False
        bounced_y = False

        out_of_limits_x = 0
        out_of_limits_y = 0

        # Publishers.
        self.vel_pub = rospy.Publisher('ref_vel', Twist, queue_size=1)

        # Dynamic reconfigure server.
        Server(BilliardConfig, self.reconf_cb)

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
        subs = [rospy.Subscriber(f'/{name}/odom_estimated', Odometry, self.odom_cb, args=i, queue_size=1)
                for name in default['robots']]
        rospy.Subscriber('imu', myIMU, self.imu_cb, queue_size=1)

        # Wait until we get poses of all robots in the system.
        while all([item == Pose2D() for item in self.poses.values()]):
            rospy.sleep(0.5)

        r = rospy.Rate(10)
        cmd_vector = Vector2()
        while not rospy.is_shutdown():
            # CONTROL
            # When the robot approaches the limits, reduce its speed.
            if self.poses[self.me].y > limits['y_top'] - self.config['reduced_buffer'] or
               self.poses[self.me].y < limits['y_bottom'] + self.config['reduced_buffer'] or
               self.poses[self.me].x > limits['x_right'] - self.config['reduced_buffer'] or
               self.poses[self.me].x < limits['x_left'] + self.config['reduced_buffer']:
                cmd_vector.set_mag(self.config['reduced_speed'])

            # When the robot exits the limited area, turn it around.
            if self.poses[self.me].y > limits['y_top']:
                if not bounced_y:
                    bounced_y = True
                    cmd_vector.set_angle(-self.poses[self.me].theta)
                    rospy.logwarn("y > top limit. New direction: %s", cmd_vector.arg())
                out_of_limits_y += 1
            elif self.poses[self.me].y < limits['y_bottom']:
                if not bounced_y:
                    bounced_y = True
                    cmd_vector.set_angle(-self.poses[self.me].theta)
                    rospy.logwarn("y < bottom limit. New direction: %s", cmd_vector.arg())
                out_of_limits_y += 1

            if self.poses[self.me].x > limits['x_right']:
                if not bounced_x:
                    bounced_x = True
                    cmd_vector.set_angle(math.pi - self.poses[self.me].theta)
                    rospy.logwarn("x > right limit. New direction: %s", cmd_vector.arg())
                out_of_limits_x += 1
            elif self.poses[self.me].x < limits['x_left']:
                if not bounced_x:
                    bounced_x = True
                    cmd_vector.set_angle(math.pi - self.poses[self.me].theta)
                    rospy.logwarn("x < left limit. New direction: %s", cmd_vector.arg())
                out_of_limits_x += 1

            # When the robot returns to the allowed area, reset flags and its speed.
            if limits['x_left'] < self.poses[self.me].x < limits['x_right']:
                bounced_x = False
                out_of_limits_x = 0
                cmd_vector.set_mag(self.config['set_speed'])
                # rospy.loginfo("X in bounds")
            if limits['y_bottom'] < self.poses[self.me].y < limits['y_top']:
                bounced_y = False
                out_of_limits_y = 0
                cmd_vector.set_mag(self.config['set_speed'])
                # rospy.loginfo("Y in bounds")

            # If the robot stays out of bounds for too long, send to towards the middle of the arena.
            if out_of_limits_x > self.config['out_samples'] or out_of_limits_y > self.config['out_samples']:
                x = random.uniform(0.3 * limits['width'], 0.7 * limits['width'])
                y = random.uniform(0.3 * limits['height'], 0.7 * limits['height'])
                direction = Vector2(x, y) - Vector2(self.poses[self.me].x, self.poses[self.me].y)
                cmd_vector.set_angle(direction.arg())
                out_of_limits_x = 0
                out_of_limits_y = 0
                

            # AVOID
            res_vector = copy(cmd_vector)
            for robot in self.poses:
                if robot != self.me:
                    delta_x = self.poses[self.me].x - self.poses[robot].x
                    delta_y = self.poses[self.me].y - self.poses[robot].y
                    delta = Vector2(delta_x, delta_y)
                    if delta.norm() < 1.5:
                        res_vector = res_vector + delta * self.config['repulsion']

            cmd_vel_msg.linear.x = res_vector.norm()
            cmd_vel_msg.linear.y = res_vector.arg()

            self.vel_pub.publish(cmd_vel_msg)

            # VISUALIZATION
            angle = Quaternion(*quaternion_from_euler(0, 0, cmd_vel_msg.linear.y))
            scale = Vector3(cmd_vel_msg.linear.x, 0.08, 0.08)

            marker.header.stamp = rospy.get_rostime()
            marker.pose.orientation = angle
            marker.scale = scale
            self.marker_pub.publish(marker)

            r.sleep()

    def odom_cb(self, msg, args):
        self.poses[args].x = msg.pose.pose.position.x
        self.poses[args].y = msg.pose.pose.position.y

    def imu_cb(self, msg):
        self.poses[self.me].theta = msg.euler.z

    def reconf_cb(self, config, level):
        self.config = config
        rospy.loginfo("Paramaters updated!")

        return config


if __name__ == "__main__":
    rospy.init_node("ref_follower")

    try:
        node = BillardController()
    except rospy.ROSInterruptException:
        pass
