#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf
import rospy
import math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from kalman_filter import KalmanFilter


def pose_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


class KalmanFilterNode(object):
    """
    ROS node implementation of Kalman filter.

    This node subscribes to a list of all existing Sphero's positions
    broadcast from OptiTrack system, associates one of them to the Sphero in
    the same namespace and uses Kalman filter to output steady position and
    velocity data for other nodes.
    """

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Initialize class variables
        self.missing_counter = 0   # Counts iterations with missing marker information
        self.pub_frequency = rospy.get_param('/ctrl_loop_freq', 10)
        self.debug_enabled = rospy.get_param('/debug_kalman', False)

        self.X_est = None
        self.filter = None
        self.initial_position = None

        # Create a publisher for commands
        pub = rospy.Publisher('odom_estimated', Odometry, queue_size=self.pub_frequency)
        if self.debug_enabled:
            # Debug publisher runs at the same frequency as incoming data.
            self.debug_pub = rospy.Publisher('kalman_debug', Odometry, queue_size=1)

        # Subscriber for the sensor
        rospy.Subscriber('pozyx/measured', TransformStamped, self.sensor_callback, queue_size=1)

        # Get the initial positions of the robots.
        self.get_initial_position()  # Get initial position
        rospy.loginfo('Initial position:\n%s\n', self.initial_position)

        # Initialize Kalman filter and estimation
        self.filter = KalmanFilter(1.0 / self.pub_frequency, self.initial_position)
        self.X_est = Odometry()
        self.X_est.pose.pose = self.initial_position

        # Create tf broadcaster
        br = tf.TransformBroadcaster()

        # Main while loop.
        rate = rospy.Rate(self.pub_frequency)
        while not rospy.is_shutdown():
            self.X_est = self.filter.predict()

            pub.publish(self.X_est)
            pos = self.X_est.pose.pose.position
            br.sendTransform((pos.x, pos.y, pos.z),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             rospy.get_namespace() + 'base_link',
                             'world')
            rospy.logdebug(' x = % 7.5f', self.X_est.pose.pose.position.x)
            rospy.logdebug(' y = % 7.5f', self.X_est.pose.pose.position.y)
            rospy.logdebug('vx = % 7.5f', self.X_est.twist.twist.linear.x)
            rospy.logdebug('vy = % 7.5f\n', self.X_est.twist.twist.linear.y)
            rate.sleep()

    def get_initial_position(self):
        """Calls service which returns Sphero's initial position."""
        while self.initial_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def sensor_callback(self, data):
        """Process received positions data and return Kalman estimation."""
        if self.initial_position is None:
            self.initial_position = data.transform.translation

        if self.filter is None:
            return

        # Get measurement.
        X_measured = data.transform.translation
        time = data.header.stamp

        # Update the filter.
        self.X_est = self.filter.update(X_measured)
        self.X_est.header.stamp = time  # TESTME: Can we remove this?

        if self.debug_enabled:
            self.debug_pub.publish(self.X_est)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Kalman')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
