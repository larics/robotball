# -*- coding: utf-8 -*-
from __future__ import division
import math
import rospy
import numpy as np
import numpy.linalg as npl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler


class KalmanFilter(object):
    """Class implementation of Kalman filter."""

    def __init__(self, step_time, position, orientation=0):
        """Initialize class variables and set initial conditions."""
        self.T = step_time
        self.X0 = self.get_numpy_state(position, orientation)
        self.P0 = 1e-3 * np.eye(3)

        self.Q = np.power(np.diag([0.1, 0.05]), 2)      # Odom variance: 0.1 rad & 5 cm
        self.R = np.power(np.diag([0.3, 0.3]), 2)       # Pozyx variance: 30 cm (x & y)
        # self.R = np.power(np.diag([0.3, 0.3, 0.1]), 2)  # Pozyx variance: 30 cm (x & y), Heading variance: 0.1 rad

        self.H = np.array([[1, 0, 0], [0, 1, 0]])
        # self.H = np.eye(3)

        self.V = np.eye(2)
        # self.V = np.eye(3)

        self.X_old = self.X0
        self.P_old = self.P0

    def get_numpy_state(self, position, orientation):
        """Convert from some type (here: ROS msg) to numpy array."""
        x = position.x
        y = position.y
        theta = orientation

        state = np.array([[x, y, theta]])
        return state.T

    def get_used_state(self, np_state):
        """Convert from numpy array to type used elsewhere (here: ROS msg)."""
        time = rospy.Time.now()
        msg = Odometry()
        msg.header.stamp = time
        msg.header.frame_id = rospy.get_namespace()
        msg.pose.pose.position.x = np_state[0][0]
        msg.pose.pose.position.y = np_state[1][0]
        msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np_state[2][0]))

        return msg

    def predict(self, v, w, ret=True):
        """Kalman's prediction phase."""
        D_dist = v * self.T
        D_theta = w * self.T
        cos_theta = math.cos(self.X_old[2][0] + D_theta)
        sin_theta = math.sin(self.X_old[2][0] + D_theta)

        X_hat_m = np.array([[self.X_old[0][0] + D_dist * cos_theta],
                            [self.X_old[1][0] + D_dist * sin_theta],
                            [self.X_old[2][0] + D_theta]
                            ])

        A = np.array([[1, 0, -D_dist * sin_theta],
                      [0, 1,  D_dist * cos_theta],
                      [0, 0,  1]
                      ])
        W = np.array([[-D_dist * sin_theta, cos_theta],
                      [ D_dist * cos_theta, sin_theta],
                      [ 1, 0]
                      ])
        P_m = A.dot(self.P_old).dot(A.T) + W.dot(self.Q).dot(W.T)

        self.X_old = X_hat_m
        self.P_old = P_m

        if ret:
            return self.get_used_state(X_hat_m)

    def update(self, Xm):
        """
        Kalman's update phase.

        Args:
            Xm: measured state vector
        """
        eye = np.eye(4)  # Identity matrix
        P_ = self.P_old  # P minus
        H = self.H
        V = self.V
        R = self.R
        X_hat_m = self.X_old  # X hat minus
        Xm = self.get_numpy_state(Xm)

        # S = H * P(-) * H^T + V * R * V^T
        # K = P(-) * H^T * S^-1
        # X(+) = X(-) + K * [y - H * X(-)]
        # P(+) = P(-) - K * S * K^T
        S = H.dot(P_).dot(H.T) + V.dot(R).dot(V.T)
        K = P_.dot(H.T).dot(npl.inv(S))
        X_hat_p = X_hat_m = K.dot(H.dot(Xm) - H.dot(X_hat_m))
        P_p = P_ - K.dot(S).dot(K.T)

        self.X_old = X_hat_p
        self.P_old = P_p

        return self.get_used_state(X_hat_p)

    def predict_update(self, Xm):
        """Call predict and update."""
        self.predict(ret=False)
        return self.update(Xm)
