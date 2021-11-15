#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import copy
import rospy
from dynamic_reconfigure.server import Server

from std_msgs.msg import ColorRGBA, Bool
from geometry_msgs.msg import Pose2D, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from robotball_path.cfg import GeometricGeneratorConfig


def make_shape(N, yaw, scale, C, D):
    yaw = math.radians(yaw)

    points = []
    for i in range(N):
        x = scale / 2 * math.cos(i * 2 * math.pi / N)
        y = scale / 2 * math.sin(i * 2 * math.pi / N)

        r_x = math.cos(yaw) * x - math.sin(yaw) * y
        r_y = math.sin(yaw) * x + math.cos(yaw) * y

        t_x = r_x + C
        t_y = r_y + D

        points.append(Pose2D(t_x, t_y, 0))

    return points


def distance_2D(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def find_closest(points, current):
    min_ind = 0
    min_val = 1000
    for i in range(len(points)):
        d = distance_2D(points[i], current)
        if d < min_val:
            min_val = d
            min_ind = i

    return min_ind


class RefGenerator(object):
    def __init__(self):
        # Class variables.
        self.show_path = False
        self.shape_params = ['N', 'yaw', 'scale', 'C', 'D']

        default = rospy.get_param('~')
        self.config = {k: default[k] for k in self.shape_params}
        self.saved_config = copy.deepcopy(self.config)

        self.odom = Odometry()
        self.ref_points = []
        self.ref_index = 0

        # Create publishers
        self.ref_pub = rospy.Publisher('pos_ref', Pose2D, queue_size=1)
        self.marker_pub = rospy.Publisher('/pos_ref_viz', Marker, queue_size=1)

        # Prepare messages.
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = rospy.get_namespace()
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale = Vector3(0.1, 0.1, 0.01)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.color = ColorRGBA(1, 0, 0, 1)
        marker.lifetime = rospy.Duration(0)
        marker.id = 0

        # Services.
        rospy.Subscriber('/draw_path', Bool, self.draw_shape)

        # Dynamic reconfigure server.
        Server(GeometricGeneratorConfig, self.reconf_cb)

        # Subscribers
        rospy.Subscriber('odom_estimated', Odometry, self.odom_cb, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            err_x = self.ref_points[self.ref_index].x - self.odom.pose.pose.position.x
            err_y = self.ref_points[self.ref_index].y - self.odom.pose.pose.position.y
            err = math.sqrt(err_x**2 + err_y**2)
            if err < 0.1:
                # We have reached the desired point.
                self.ref_index = (self.ref_index + 1) % len(self.ref_points)

                rospy.loginfo("Current reference reached.")
                rospy.loginfo("New reference:\n%s", self.ref_points[self.ref_index])
                rospy.sleep(1)

            self.ref_pub.publish(self.ref_points[self.ref_index])

            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = self.ref_points[self.ref_index].x
            marker.pose.position.y = self.ref_points[self.ref_index].y
            self.marker_pub.publish(marker)

            rate.sleep()

    def odom_cb(self, msg):
        self.odom = msg

    def reconf_cb(self, config, level):
        rospy.loginfo("Shape update\n"
                      "N: {N}, yaw: {yaw}, scale: {scale:.3f}, C: {C:.1f}, D: {D:.1f}".format(**self.saved_config))

        new_config = {k: config[k] for k in self.shape_params}
        self.ref_points = make_shape(**new_config)
        self.ref_index = find_closest(self.ref_points, self.odom.pose.pose.position)

        self.draw_shape()

        return config

    def draw_shape(self, msg=None):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = rospy.get_namespace() + 'path'
        marker.type = Marker.LINE_STRIP
        marker.scale = Vector3(0.1, 0, 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.color = ColorRGBA(0, 1, 0, 1)
        marker.points = [Point(p.x, p.y, 0) for p in self.ref_points + [self.ref_points[0]]]
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        # Parameter triggered update
        if msg is None and self.show_path is False:
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(5)
        # Message triggered update
        if msg is not None:
            if msg.data is True:
                self.show_path = True
                marker.action = Marker.ADD
                marker.lifetime = rospy.Duration(0)
            else:
                self.show_path = False
                marker.action = Marker.DELETE
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("ref_generator")

    try:
        node = RefGenerator()
    except rospy.ROSInterruptException:
        pass
