#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import copy
import collections
import numpy as np
import rospy

from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Vector3, Quaternion, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray

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
        marker.header.frame_id = 'world'
        marker.ns = rospy.get_namespace()
        trail_length = 2
        trail_spacing = 0.1
        trail_number = int(round(trail_length / trail_spacing))

        ## CYLINDER TRAIL
        # self.marker_pub = rospy.Publisher('/odom_viz', Marker, queue_size=1)
        # marker.type = Marker.CYLINDER
        # marker.action = Marker.ADD
        # marker.scale = Vector3(0.35, 0.35, 0.05)
        # marker.color = ColorRGBA(1, 1, 1, 0.3)

        ## RING TRAIL
        # self.marker_pub = rospy.Publisher('/odom_viz', MarkerArray, queue_size=1)
        # markers = MarkerArray()
        # marker.type = Marker.LINE_STRIP
        # marker.action = Marker.ADD
        # marker.scale = Vector3(0.01, 0, 0)
        # marker.color = ColorRGBA(1, 1, 1, 0.5)
        # marker.points = [Point(0.175 * math.cos(t), 0.175 * math.sin(t), 0) for t in np.linspace(0, 2 * math.pi, 50)]
        # point_list = collections.deque(maxlen=trail_number)

        ## LINE TRAIL
        # self.marker_pub = rospy.Publisher('/odom_viz', Marker, queue_size=1)
        # marker.type = Marker.LINE_STRIP
        # marker.action = Marker.ADD
        # marker.pose = Pose(Point(), Quaternion(0, 0, 0, 1))
        # marker.scale = Vector3(0.35, 0, 0)
        # marker.color = ColorRGBA(1, 1, 1, 0.5)
        # marker.lifetime = rospy.Duration(2)
        # point_list = collections.deque(maxlen=trail_number)

        ## CMD_VEL VECTOR
        self.marker_pub = rospy.Publisher('/odom_viz', Marker, queue_size=1)
        marker.header.frame_id = rospy.get_namespace() + 'base_link'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.color = ColorRGBA(0, 1, 1, 1)
        marker.pose = Pose()
        marker.pose.position.z = 0.1776  # Sphero radius
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True

        # Subscribers.
        rospy.Subscriber('pos_ref', Pose2D, self.ref_cb, queue_size=1)
        rospy.Subscriber('odom_estimated', Odometry, self.odom_cb, queue_size=1)

        r = rospy.Rate(rate)
        cum_dist = 0
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

            # dist, angle = self.get_relative_motion()
            # cum_dist += dist
            # if cum_dist > trail_spacing:
            #     ## CYLINDER TRAIL
            #     # marker.pose = self.odom.pose.pose
            #     # marker.pose.orientation = Quaternion(*quaternion_from_euler( math.pi / 2, 0, angle + math.pi / 2))
            #     # marker.pose.position.z = 0.175
            #     # marker.id = (marker.id + 1) % trail_number
            #     # self.marker_pub.publish(marker)

            #     ## RING TRAIL
            #     # marker.pose = self.odom.pose.pose
            #     # marker.pose.orientation = Quaternion(*quaternion_from_euler(math.pi / 2, 0, angle + math.pi / 2))
            #     # marker.pose.position.z = 0.175
            #     # marker.id = (marker.id + 1) % trail_number
            #     # point_list.append(marker)
            #     # markers.markers = list(point_list)
            #     # self.marker_pub.publish(markers)

            #     ## LINE TRAIL
            #     # point_list.append(Point(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0.175))
            #     # marker.points = list(point_list)
            #     # self.marker_pub.publish(marker)

            #     cum_dist = 0

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
    rospy.init_node("follower")

    try:
        node = RefTracker()
    except rospy.ROSInterruptException:
        pass
