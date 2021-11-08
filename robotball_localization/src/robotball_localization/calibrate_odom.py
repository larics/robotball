#!/usr/bin/env python3

import rospy
import math
import numpy as np
from scipy.optimize import least_squares
from collections import namedtuple

from geometry_msgs.msg import Vector3, Twist
from robotball_msgs.msg import Status, Debug

Params = namedtuple('Params', ['l_r', 'r_r', 'ws', 't'])


def calibrate_linear(k, *args):
    def update_odometry(old_odom, wl, wr, dt, k, p):
        vt = (wl * p.l_r * k[0] + wr * p.r_r * k[1]) / 2 * p.t
        w = (-wl * p.l_r * k[0] + wr * p.r_r * k[1]) / (p.ws * k[2])
        D = vt * dt
        delta_theta = w * dt
        new_theta = old_odom[0][2] + delta_theta
        delta_x = D * math.cos(new_theta)
        delta_y = D * math.sin(new_theta)
        odometry = old_odom + np.array([[delta_x, delta_y, delta_theta]])
        return odometry

    odom_history = args[0]
    final_pose = args[1]
    robot_params = args[2]
    odom = np.array([[0, 0, 0]])

    for i in range(np.size(odom_history, 1)):
        odom = update_odometry(odom, odom_history[0][i], odom_history[1][i], odom_history[2][i], k, robot_params)

    return np.reshape(np.abs(odom - final_pose), 3)


def calibrate_rotation(k, *args):

    def update_odometry(old_odom, wl, wr, dt, k, p):
        w = (-wl * p.l_r * k[0] + wr * p.r_r * k[1]) / (p.ws * k[2])
        delta_theta = w * dt
        odometry = old_odom + delta_theta
        return odometry

    odom_history = args[0]
    final_pose = args[1]
    lin_k = args[2]
    robot_params = args[3]
    odom = 0

    k = [lin_k[0], lin_k[1], k[0]]
    for i in range(np.size(odom_history, 1)):
        odom = update_odometry(odom, odom_history[0][i], odom_history[1][i], odom_history[2][i], k, robot_params)

    return abs(odom - final_pose)


class Calibration(object):
    def __init__(self):
        self.active_step = 'none'
        self.wl = []
        self.wr = []
        self.dt = []
        self.final_odom = None
        self.final_true = None
        self.last_msg = 0
        self.robot_params = None

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('status', Status, self.status_cb, queue_size=1)
        rospy.Subscriber('debug', Debug, self.debug_cb, queue_size=1)

        while self.last_msg == 0 or rospy.get_time() - self.last_msg < 5:
            if not rospy.is_shutdown():
                rospy.sleep(0.5)

        ######################################
        input("\nREADY FOR LINEAR CALIBRATION?")
        self.active_step = "lin"
        for i in range(5):
            self.cmd_pub.publish(linear=Vector3(1, 0, 0))
        self.last_msg = rospy.get_time()
        # We are receiving messages.
        while self.active_step == 'lin' and rospy.get_time() - self.last_msg < 5:
            rospy.sleep(0.5)
        # Final message has arrived, let's calculate the coefficients.
        # But first, we need to know the final position of the robot.
        self.final_true[0] = float(input("Enter the value of x coordinate: "))
        self.final_true[1] = float(input("Enter the value of y coordinate: "))

        odom_history = np.array([self.wl, self.wr, self.dt])
        final_pose = np.array([self.final_true])
        k_init = np.array([1, 1, 1])
        error = calibrate_linear(k_init, odom_history, final_pose, self.robot_params)
        print("Error before LINEAR calibration: {}".format(error))
        k_lin = least_squares(calibrate_linear, k_init, args=(odom_history, final_pose, self.robot_params))
        error = calibrate_linear(k_lin.x, odom_history, final_pose, self.robot_params)
        print("Error after LINEAR calibration: {}".format(error))
        print("Final calibration parameters: \n {}\n".format(k_lin.x))

        ######################################
        input("READY FOR ROTATION CALIBRATION?")
        self.active_step = "rot"
        for i in range(5):
            self.cmd_pub.publish(linear=Vector3(0, 1, 0))
        self.last_msg = rospy.get_time()
        # We are receiving messages for ***ROTATION*** calibration.
        while self.active_step == 'rot' and rospy.get_time() - self.last_msg < 5:
            rospy.sleep(0.5)
        # Final message has arrived, let's calculate the coefficients.
        odom_history = np.array([self.wl, self.wr, self.dt])
        final_pose = np.array([self.final_true])
        k_init = np.array([k_lin.x[2]])
        error = calibrate_rotation(k_init, odom_history, final_pose, k_lin.x[0:2], self.robot_params)
        print("Error before ROTATION calibration: {}".format(error))
        k_rot = least_squares(calibrate_rotation, k_init, args=(odom_history, final_pose, k_lin.x[0:2], self.robot_params))
        error = calibrate_rotation(k_init, odom_history, final_pose, k_lin.x[0:2], self.robot_params)
        print("Error after ROTATION calibration: {}".format(error))
        print("Final calibration parameters: \n {}\n".format(k_rot.x))

    def status_cb(self, data):
        # Calibration status
        calibration = str(data.calibration)
        if calibration[0] == '1':
            rospy.loginfo("Calibraton status> SYS: %s, GYR: %s, ACC: %s, MAG: %s", *calibration[1:])
        elif calibration[0] == '9':
            rospy.loginfo_once("Calibraton complete.")
        self.last_msg = rospy.get_time()

    def debug_cb(self, data):
        if self.active_step == 'none':
            self.robot_params = Params(l_r=data.odom.pos.left,
                                       r_r=data.odom.pos.right,
                                       ws=data.odom.omega.left,
                                       t=data.odom.omega.right)
            rospy.loginfo_once("robot_params: \n %s", self.robot_params)


        if self.active_step == 'lin':
            self.wl.append(data.odom.omega.left)
            self.wr.append(data.odom.omega.right)
            self.dt.append(data.odom.time)
            self.final_odom = [data.odom.pos.left, data.odom.pos.right, data.hdg.measured]
            self.final_true = [0, 0, data.hdg.setpoint]
            self.last_msg = rospy.get_time()

        if self.active_step == "rot":
            self.wl.append(data.odom.omega.left)
            self.wr.append(data.odom.omega.right)
            self.dt.append(data.odom.time)
            self.final_odom = data.odom.pos.left
            self.final_true = data.odom.pos.right
            self.last_msg = rospy.get_time()


if __name__ == "__main__":
    rospy.init_node("calibration", anonymous=False)

    try:
        node = Calibration()
    except rospy.ROSInterruptException:
        pass
