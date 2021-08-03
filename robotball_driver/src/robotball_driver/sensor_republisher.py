#!/usr/bin/env python3

import re
import math
import rospy
import serial

from std_msgs.msg import Float32, ColorRGBA, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Point, Vector3

from dynamic_reconfigure.server import Server
from robotball_driver.cfg import PIDConfig
from robotball_driver.msg import MeasuredMsg, SetpointMsg, OutputMsg, IMU


class SensorRepublisher(object):
    def __init__(self):
        self.arduino = serial.Serial("/dev/ttyUSB0", baudrate=115200)
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()
        in_data = None

        self.raw_pub = rospy.Publisher('raw', IMU, queue_size=1)
        self.cal_pub = rospy.Publisher('cal', IMU, queue_size=1)

        rate = rospy.Rate(100);
        last_published = rospy.get_time()
        while not rospy.is_shutdown():
            
            try:
                # in_data = self.arduino.read(self.arduino.in_waiting).decode('utf-8')
                in_data = self.arduino.read_until(b'\r\n').decode('utf-8')
            except Exception:
                print("Couldn't decode!")

            if in_data:
                self.parse_data(in_data)
            rate.sleep()


    def parse_data(self, data):
        # print(data)

        valid_search = re.search('^(Calibrated|Raw): ((?:[-0-9.]{6,},?){9})', data)
        if valid_search is not None:
            entry = valid_search.group(1)
            try:
                payload = list(map(float, valid_search.group(2).split(',')))
            except ValueError:
                return

            msg = IMU()
            msg.accel = Vector3(*payload[0:3])
            msg.gyro = Vector3(*payload[3:6])
            msg.mag = Vector3(*payload[6:9])
            msg.header.stamp = rospy.Time.now()

            if entry == 'Calibrated':
                self.cal_pub.publish(msg)
            elif entry == 'Raw':
                self.raw_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("sensors", anonymous = False)

    try:
        node = SensorRepublisher()
    except rospy.ROSInterruptException:
        pass
