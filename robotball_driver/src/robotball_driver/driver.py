#!/usr/bin/env python3

import math
import rospy

from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
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
        self.mode_manual = True
        self.latest_config = None
        self.calibration_complete = False
        rospy.sleep(1)

        # Publishers
        self.cmd_vel = Twist()
        self.cmd_vel_pub = rospy.Publisher('man_vel', Twist, queue_size=1)
        self.dyn_reconf_pub = rospy.Publisher('dyn_reconf', DynReconf, queue_size=1, latch=True)

        # Subscribers
        # rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)
        # rospy.Subscriber('imu', IMU, self.imu_cb, queue_size=1)
        rospy.Subscriber('status', Status, self.status_cb, queue_size=1)
        # rospy.Subscriber('debug', Debug, self.debug_cb, queue_size=1)

        # Dynamic reconfigure server for PID tuning
        self.server = Server(PIDConfig, self.reconfigure_callback)
        self.client = Client('driver', timeout=2.0)

        # Joystick control
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode_manual:
                self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def status_cb(self, data):
        # Calibration status
        calibration = str(data.calibration)
        if calibration[0] == '1' and not self.calibration_complete:
            rospy.loginfo("Calibraton status> SYS: %s, GYR: %s, ACC: %s, MAG: %s", *calibration[1:])
        elif calibration[0] == '9':
            rospy.loginfo_once("Calibraton complete.")
            self.calibration_complete = True

        # Battery status
        # TODO

    def joy_callback(self, data):
        """Receive inputs from joystick."""

        # Green "A" button: Toggle automatic and manual mode.
        if data.buttons[1]:
            self.mode_manual = not self.mode_manual
            rospy.loginfo("Switched to %s mode.", "MANUAL" if self.mode_manual else "AUTOMATIC")

        # D-pad: Step speed inputs.
        if data.axes[5] == 1:
            magnitude = 30 / 45
        elif data.axes[5] == -1:
            magnitude = -30 / 45
        # Left stick: Gradual speed inputs.
        else:
            magnitude = data.axes[1]

        # Right stick: Gradual direction inputs.
        if data.axes[2] != 0 or data.axes[3] != 0:
            direction = wrap_pi_pi(math.atan2(data.axes[3], -data.axes[2]) - math.pi / 2)
            self.last_direction = direction
        # D-pad: Step direction inputs.
        else:
            if data.axes[4] == 1:
                direction = math.pi / 2
            elif data.axes[4] == -1:
                direction = -math.pi / 2
            else:
                direction = 0

        # Package the commanded velocity message.
        # Blue "X" button: Reset odometry.
        self.cmd_vel.linear = Vector3(magnitude, direction, data.buttons[0])

        # "L1" button: Enable/disable heading controller.
        if data.buttons[4]:
            self.client.update_configuration({"hdg_enabled": not self.latest_config.hdg.enabled})
        # "R1" button: Enable/disable velocity controller.
        if data.buttons[5]:
            self.client.update_configuration({"vel_enabled": not self.latest_config.speed.enabled})

        # Press back and start simultaneously to reset the Arduino.
        if data.buttons[8] and data.buttons[9]:
            rospy.wait_for_service('serial_node/reset_arduino')
            reset = rospy.ServiceProxy('serial_node/reset_arduino', Empty)
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
        msg.hdg_offset = config['offset']
        self.dyn_reconf_pub.publish(msg)
        self.latest_config = msg

        # Store PID values
        if self.first_pass:
            rospy.sleep(1)
            self.first_pass = False

        return config


def shutdown_hook():
    rospy.logwarn("MAKE SURE TO ORIENTATE THE ROBOT UPRIGHT BEFORE SHUTTING DOWN.")


if __name__ == "__main__":
    rospy.init_node("driver", anonymous=False)
    rospy.on_shutdown(shutdown_hook)

    try:
        node = Driver()
    except rospy.ROSInterruptException:
        pass
