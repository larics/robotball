#!/usr/bin/env python3

import math
import rospy

from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from robotball_msgs.msg import Odometry, IMU, Status, Debug, DynReconf

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from robotball_driver.cfg import PIDConfig


def wrap_0_2pi(x):
    return math.fmod(2 * math.pi + math.fmod(x, 2 * math.pi), 2 * math.pi)


def wrap_pi_pi(x):
    return -math.pi + wrap_0_2pi(x + math.pi)


class Driver(object):
    def __init__(self):
        self.first_pass = True
        self.latest_config = None
        rospy.sleep(1)

        # Publishers
        self.cmd_vel = Vector3()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Vector3, queue_size=1)
        self.dyn_reconf_pub = rospy.Publisher('dyn_reconf', DynReconf, queue_size=1, latch=True)

        # Subscribers
        # rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)
        # rospy.Subscriber('imu', IMU, self.imu_cb, queue_size=1)
        rospy.Subscriber('status', Status, self.status_cb, queue_size=1)
        # rospy.Subscriber('debug', Debug, self.debug_cb, queue_size=1)

        # Dynamic reconfigure server for PID tuning
        self.server = Server(PIDConfig, self.reconfigure_callback)
        self.client = Client('robotball_driver', timeout=2.0)

        # Joystick control
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def status_cb(self, data):
        # Calibration status
        calibration = str(data.calibration)
        if calibration[0] == '1':
            rospy.loginfo("Calibraton status> SYS: %s, GYR: %s, ACC: %s, MAG: %s", *calibration[1:])
        elif calibration[0] == '9':
            rospy.loginfo_once("Calibraton complete.")

        # Battery status
        # TODO

    def joy_callback(self, data):
        """Receive inputs from joystick."""

        # D-pad for step speed inputs.
        if data.axes[5] == 1:
            magnitude = 30 / 45
        elif data.axes[5] == -1:
            magnitude = -30 / 45
        # Left stick for gradual speed inputs.
        else:
            magnitude = data.axes[1]

        # Right stick for gradual direction inputs.
        if data.axes[2] != 0 or data.axes[3] != 0:
            direction = wrap_pi_pi(math.atan2(data.axes[3], -data.axes[2]) - math.pi / 2)
            self.last_direction = direction
        # D-pad for step direction inputs.
        else:
            if data.axes[4] == 1:
                direction = math.pi / 2
            elif data.axes[4] == -1:
                direction = -math.pi / 2
            else:
                direction = 0

        self.cmd_vel = Vector3(magnitude, direction, 0)

        # Enable/disable heading controller.
        if data.buttons[4]:
            self.client.update_configuration({"hdg_enabled": not self.latest_config.hdg.enabled})
        # Enable/disable velocity controller.
        if data.buttons[5]:
            self.client.update_configuration({"vel_enabled": not self.latest_config.speed.enabled})

        # Press back and start simultaneously to reset the Arduino.
        if data.buttons[8] and data.buttons[9]:
            rospy.wait_for_service('/serial_node/reset_arduino')
            reset = rospy.ServiceProxy('/serial_node/reset_arduino', Empty)
            try:
                rospy.logwarn("Resetting arduino now!")
                resp = reset()
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: %s", exc)

    def reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request:
            Velocity: {vel_enabled}
            vel_P: {vel_P}
            vel_I: {vel_I}
            vel_D: {vel_D}

            Pitch: {pitch_enabled}
            pitch_P: {pitch_P}
            pitch_I: {pitch_I}
            pitch_D: {pitch_D}

            Heading: {hdg_enabled}
            hdg_P: {hdg_P}
            hdg_I: {hdg_I}
            hdg_D: {hdg_D}
            """.format(**config))

        msg = DynReconf()
        msg.speed.enabled = config['vel_enabled']
        msg.speed.P = config['vel_P']
        msg.speed.I = config['vel_I']
        msg.speed.D = config['vel_D']
        msg.pitch.enabled = config['pitch_enabled']
        msg.pitch.P = config['pitch_P']
        msg.pitch.I = config['pitch_I']
        msg.pitch.D = config['pitch_D']
        msg.hdg.enabled = config['hdg_enabled']
        msg.hdg.P = config['hdg_P']
        msg.hdg.I = config['hdg_I']
        msg.hdg.D = config['hdg_D']
        self.dyn_reconf_pub.publish(msg)
        self.latest_config = msg

        # Store PID values
        if self.first_pass:
            rospy.sleep(1)
            self.first_pass = False

        return config


if __name__ == "__main__":
    rospy.init_node("driver", anonymous=False)

    try:
        node = Driver()
    except rospy.ROSInterruptException:
        pass
