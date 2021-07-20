#!/usr/bin/env python3

import math
import rospy
import serial

from std_msgs.msg import Float32, ColorRGBA, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Point

from dynamic_reconfigure.server import Server
from robotball_driver.cfg import PIDConfig


def wrap_0_2pi(x):
    return math.fmod(2*math.pi + math.fmod(x, 2*math.pi), 2*math.pi);

def wrap_pi_pi(x):
    return -math.pi + wrap_0_2pi(x + math.pi);

class DirectDrive(object):
    def __init__(self):
        self.arduino = serial.Serial("/dev/ttyUSB0", baudrate=115200)
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()
        in_data = None

        self.setpoint_msg = Float32()
        self.measured_msg = Float32()
        self.output_msg = Float32()
        self.speed_msg = Float32()
        self.setpoint_pub = rospy.Publisher('setpoint', Float32, queue_size=1)
        self.measured_pub = rospy.Publisher('measured', Float32, queue_size=1)
        self.output_pub = rospy.Publisher('output', Float32, queue_size=1)
        self.speed_pub = rospy.Publisher('speed', Float32, queue_size=1)

        # PID tuning
        self.PID_types = ['vel', 'pitch', 'hdg']
        self.PID_values = {key: {} for key in self.PID_types}
        self.srv = Server(PIDConfig, self.reconfigure_callback)

        # Joystick control
        self.dataToWrite = None
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

        rate = rospy.Rate(100);
        last_published = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.dataToWrite is not None and (now - last_published > 0.1):
                self.arduino.write(bytes(self.dataToWrite, 'utf-8'))
                last_published = now
            
            try:
                in_data = self.arduino.read(self.arduino.in_waiting).decode('utf-8')
            except Exception:
                print("Couldn't decode!")

            if in_data:
                self.parse_data(in_data)
            rate.sleep()


    def parse_data(self, data):
        line = data.split('\r\n')
        if len(line) < 3:
            return
        fields = line[-2].split(' | ')

        self.setpoint_msg.data = float(fields[2].split()[1])
        self.measured_msg.data = float(fields[3].split()[1])
        self.output_msg.data = float(fields[5].split()[1])
        self.speed_msg.data = float(fields[7].split()[1])

        self.setpoint_pub.publish(self.setpoint_msg)
        self.measured_pub.publish(self.measured_msg)
        self.output_pub.publish(self.output_msg)
        self.speed_pub.publish(self.speed_msg)

        print(line[-2])


    def reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request:
            vel_P: {vel_P}
            vel_I: {vel_I}
            vel_D: {vel_D}

            pitch_P: {pitch_P}
            pitch_I: {pitch_I}
            pitch_D: {pitch_D}

            hdg_P: {hdg_P}
            hdg_I: {hdg_I}
            hdg_D: {hdg_D}
            """.format(**config))

        # Store PID values
        values = '<'
        for t in self.PID_types:
            for v in ['P', 'I', 'D']:
                self.PID_values[t][v] = config['{}_{}'.format(t, v)]
                values += str(self.PID_values[t][v]) + ';'
        values = values[:-1]+'>'
        print(values)
        self.arduino.write(bytes(values, 'utf-8'))

        return config

    def joy_callback(self, data):
        """Receive inputs from joystick."""

        if data.axes[5] == 1:
            magnitude = 0.5
        elif data.axes[5] == -1:
            magnitude = -0.5
        else:
            magnitude = data.axes[1]


        if data.axes[2] != 0 or data.axes[3] != 0:
            direction = wrap_pi_pi(math.atan2(data.axes[3], -data.axes[2]) - math.pi / 2)
            self.last_direction = direction
        else:
            direction = 0

        self.dataToWrite = '['+str(round(magnitude, 5))+';'+str(round(direction, 5))+']'
        # print(self.dataToWrite)

if __name__ == "__main__":
    rospy.init_node("PID_tuning", anonymous = False)

    try:
        node = DirectDrive()
    except rospy.ROSInterruptException:
        pass
