#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf_conversions as tfc

from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3, TransformStamped
from robotball_msgs.msg import Odometry, IMU, Status, DynReconf

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
        self.enabled = False

        self.position = Vector3()
        self.last_direction = 0

        rospy.sleep(1)

        # Publishers
        self.cmd_vel = Twist()
        self.cmd_vel_pub = rospy.Publisher('man_vel', Twist, queue_size=1)
        self.dyn_reconf_pub = rospy.Publisher('dyn_reconf', DynReconf, queue_size=1, latch=True)

        # Create tf broadcaster
        self.tf_br = tf2_ros.TransformBroadcaster()

        # Dynamic reconfigure server for PID tuning
        self.server = Server(PIDConfig, self.reconfigure_callback)
        self.client = Client('driver', timeout=2.0)

        # Subscribers
        rospy.Subscriber('odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('imu', IMU, self.imu_cb, queue_size=1)
        rospy.Subscriber('status', Status, self.status_cb, queue_size=1)
        # rospy.Subscriber('debug', Debug, self.debug_cb, queue_size=1)

        # Joystick control
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode_manual:
                self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def odom_cb(self, data):
        self.position = Vector3(data.pose.x, data.pose.y, 0.1776)  # Radius of the sphere

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'odom'
        t.child_frame_id = rospy.get_namespace() + 'wheelbase'
        t.transform.translation = self.position
        q = tfc.transformations.quaternion_from_euler(0, 0, data.pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)

    def imu_cb(self, data):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'odom'
        t.child_frame_id = rospy.get_namespace() + 'IMU'
        t.transform.translation = self.position
        t.transform.rotation = data.orientation
        self.tf_br.sendTransform(t)

    def status_cb(self, data):
        # Calibration status
        calibration = str(data.calibration)
        if calibration[0] == '1':
            rospy.loginfo("Calibraton status> SYS: %s, GYR: %s, ACC: %s, MAG: %s", *calibration[1:])
        elif calibration[0] == '9' and calibration[1:] != '0000':
            rospy.loginfo_once("Calibraton complete.")
        else:
            rospy.logwarn("Lost connection to the IMU!")

        # Battery status
        # TODO

    def joy_callback(self, data):
        """Receive inputs from joystick."""

        if not self.enabled:
            return

        # Green "A" button: Toggle automatic and manual mode.
        if data.buttons[1]:
            self.mode_manual = not self.mode_manual
            rospy.loginfo("Switched to %s mode.", "MANUAL" if self.mode_manual else "AUTOMATIC")

        # Left stick: Gradual speed inputs.
        magnitude = data.axes[1]
        if not self.latest_config.hdg.enabled:
            direction = data.axes[0]
        # D-pad: Step speed inputs.
        if data.axes[5] == 1:
            magnitude = 30 / 45
        elif data.axes[5] == -1:
            magnitude = -30 / 45

        if self.latest_config.hdg.enabled:
            # Right stick: Gradual direction inputs.
            if data.axes[2] != 0 or data.axes[3] != 0:
                direction = wrap_pi_pi(math.atan2(data.axes[3], -data.axes[2]))
            # D-pad: Step direction inputs.
            else:
                if data.axes[4] == 1:
                    direction = math.pi / 2
                elif data.axes[4] == -1:
                    direction = -math.pi / 2
                else:
                    direction = self.last_direction
            self.last_direction = direction

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
                reset()
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

            joystick: {joystick}
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

        self.enabled = config['joystick']
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
