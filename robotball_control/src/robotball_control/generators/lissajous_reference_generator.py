#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import copy
import numpy as np
import scipy.integrate as integrate
import fractions

import rospy
from dynamic_reconfigure.server import Server

from geometry_msgs.msg import Pose2D, Vector3, Quaternion, Point
from std_msgs.msg import ColorRGBA, Bool
from visualization_msgs.msg import Marker

from robotball_control.cfg import LissajousGeneratorConfig


def lissajous(t, config, scale=1):
    x = config['A'] * math.sin(config['a'] * scale * t + config['f']) + config['C']
    y = config['B'] * math.sin(config['b'] * scale * t) + config['D']

    reset = abs((scale * t) % (2 * math.pi)) <= 0.1
    return x, y, reset


class RefGenerator(object):
    def __init__(self):
        # Class variables.
        self.show_path = False
        self.trail_length = 10
        self.liss_params = ['a', 'b', 'f', 'A', 'B', 'C', 'D', 'Speed', 'Sync']

        default = rospy.get_param('~')
        self.config = {k: default[k] for k in self.liss_params}
        self.saved_config = copy.deepcopy(self.config)
        self.v = None
        self.saved_v = 1

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

        # Services.
        rospy.Subscriber('/draw_path', Bool, self.draw_lissajous)

        # Dynamic reconfigure server.
        Server(LissajousGeneratorConfig, self.reconf_cb)

        self.v = self.saved_v
        pose_ref = Pose2D()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            x, y, reset = lissajous(rospy.get_time(), self.config, self.v)

            # Update path
            if reset and self.config != self.saved_config:
                rospy.loginfo("Changing the path NOW.")
                self.draw_lissajous()
                self.config = copy.deepcopy(self.saved_config)
                self.v = self.saved_v

            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.lifetime = rospy.Duration(self.trail_length)
            marker.id = (marker.id + 1) % (self.trail_length * 10 + 1)
            self.marker_pub.publish(marker)

            pose_ref.x = x
            pose_ref.y = y
            self.ref_pub.publish(pose_ref)

            rate.sleep()

    def reconf_cb(self, config, level):
        # Visualization params
        self.trail_length = config['Trail']

        # Reduce a/b fraction.
        ab = fractions.Fraction(config['a'], config['b'])
        a = ab.numerator
        b = ab.denominator

        # Check parameter validity to avoid non-smooth paths.
        if abs(config['f'] - 1.57) <= 0.01:
            if a % 2 == 0:
                rospy.logwarn("When 'f' is pi/2, 'a' must be odd.")
                return config
        elif abs(config['f'] - 0) <= 0.01 or abs(config['f'] - 3.14) <= 0.01:
            if (a + b) % 2 == 0 and a != b:
                rospy.logwarn("When 'f' is 0 or pi, 'a+b' must be odd.")
                return config

        self.saved_config = {k: config[k] for k in self.liss_params}
        self.saved_config['a'] = a
        self.saved_config['b'] = b

        rospy.loginfo("Lissajous update\n"
                      "a: {a}, b: {b}, f: {f:.3f}, A: {A:.1f}, B: B{B:.1f}, C: {C:.1f}, D: {D:.1f}, v: {Speed:.1f}"
                      .format(**self.saved_config))
        self.scale_lissajous()
        self.draw_lissajous()

        return config

    def draw_lissajous(self, msg=None):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = rospy.get_namespace() + 'path'
        marker.type = Marker.LINE_STRIP
        marker.scale = Vector3(0.1, 0, 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.color = ColorRGBA(0, 1, 0, 1)
        liss_gen = (lissajous(t, self.saved_config) for t in np.linspace(0, 2 * math.pi, 100))
        marker.points = [Point(x, y, 0) for x, y, _ in liss_gen]
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

    def scale_lissajous(self):
        if self.saved_config['Sync']:
            a = 1
            b = 1
        else:
            a = self.saved_config['a']
            b = self.saved_config['b']
        f = self.saved_config['f']
        A = self.saved_config['A']
        B = self.saved_config['B']

        def int(x):
            return math.sqrt((a * A * math.cos(a * x + f))**2 + (b * B * math.cos(b * x))**2)

        result = integrate.quad(int, 0, 2 * math.pi)
        speed = result[0] / (2 * math.pi)
        self.saved_v = self.saved_config['Speed'] / speed


if __name__ == "__main__":
    rospy.init_node("ref_generator")

    try:
        node = RefGenerator()
    except rospy.ROSInterruptException:
        pass
