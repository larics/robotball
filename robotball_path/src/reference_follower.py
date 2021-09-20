#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import copy
import collections

import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Pose2D, Vector3, Quaternion, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# from robotball_path.cfg import GeneratorConfig


class RefTracker(object):
    def __init__(self):
        # Variables.
        self.reference = Pose2D()
        self.odom = Odometry()
        self.odom_old = Odometry()

        # Publishers.
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

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
        self.marker_pub = rospy.Publisher('/odom_viz_a', MarkerArray, queue_size=1)
        markers = MarkerArray()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(0.01, 0, 0)
        marker.color = ColorRGBA(1, 1, 1, 0.5)
        marker.points = [Point(0.175 * math.cos(t), 0.175 * math.sin(t), 0) for t in np.linspace(0, 2 * math.pi, 50)]
        point_list = collections.deque(maxlen=trail_number)

        ## LINE TRAIL
        # self.marker_pub = rospy.Publisher('/odom_viz', Marker, queue_size=1)
        # marker.type = Marker.LINE_STRIP
        # marker.action = Marker.ADD
        # marker.pose = Pose(Point(), Quaternion(0, 0, 0, 1))
        # marker.scale = Vector3(0.35, 0, 0)
        # marker.color = ColorRGBA(1, 1, 1, 0.5)
        # marker.lifetime = rospy.Duration(2)
        # point_list = collections.deque(maxlen=trail_number)

        # Subscribers.
        rospy.Subscriber('pos_ref', Pose2D, self.ref_cb, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)

        rate = rospy.Rate(50)
        cum_dist = 0
        while not rospy.is_shutdown():
            cmd_vel = self.compute()
            self.vel_pub.publish(cmd_vel)

            dist, angle = self.get_relative_motion()
            cum_dist += dist
            if cum_dist > trail_spacing:
                ## CYLINDER TRAIL
                # marker.pose = self.odom.pose.pose
                # marker.pose.orientation = Quaternion(*quaternion_from_euler( math.pi / 2, 0, angle + math.pi / 2))
                # marker.pose.position.z = 0.175
                # marker.id = (marker.id + 1) % trail_number
                # self.marker_pub.publish(marker)

                ## RING TRAIL
                marker.pose = self.odom.pose.pose
                marker.pose.orientation = Quaternion(*quaternion_from_euler(math.pi / 2, 0, angle + math.pi / 2))
                marker.pose.position.z = 0.175
                marker.id = (marker.id + 1) % trail_number
                point_list.append(marker)
                markers.markers = list(point_list)
                self.marker_pub.publish(markers)

                ## LINE TRAIL
                # point_list.append(Point(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0.175))
                # marker.points = list(point_list)
                # self.marker_pub.publish(marker)

                cum_dist = 0

            rate.sleep()

    def ref_cb(self, msg):
        self.reference = msg

    def odom_cb(self, msg):
        self.odom_old = copy.deepcopy(self.odom)
        self.odom = msg

    def get_relative_motion(self):
        pos_new = self.odom.pose.pose.position
        pos_old = self.odom_old.pose.pose.position

        dist = math.sqrt((pos_new.x - pos_old.x) ** 2 + (pos_new.y - pos_old.y) ** 2)
        angle = math.atan2(pos_new.y - pos_old.y, pos_new.x - pos_old.x)

        return dist, angle

    def compute(self):
        cmd_vel = Twist()
        radius = 0.5
        max_force = 1

        err_x = self.reference.x - self.odom.pose.pose.position.x
        err_y = self.reference.y - self.odom.pose.pose.position.y

        cmd_vel.linear.x = max_force * min(err_x / radius, 1)
        cmd_vel.linear.y = max_force * min(err_y / radius, 1)

        return cmd_vel


if __name__ == "__main__":
    rospy.init_node("follower")

    try:
        node = RefTracker()
    except rospy.ROSInterruptException:
        pass
