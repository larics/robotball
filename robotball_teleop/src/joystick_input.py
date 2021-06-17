#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from random import random, randint, seed
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, ColorRGBA, Bool


class JoystickInputNode(object):
    """
    Node subscribes to joystick input data.
    `joystick_callback function gets called each time any of the
    joystick buttons or axes change states.
    """

    def __init__(self):
        """Create subscribers and publishers and initialize class variables."""

        global arduino
        arduino = serial.Serial("/dev/ttyUSB0", baudrate=115200)

        # Create a subscriber
        rospy.Subscriber("vel", Twist, self.callback, queue_size=1)
        
        rospy.spin()

    def callback(self, data):
        """Receive inputs from joystick."""

        pitch_sp = data.linear.y
        heading_sp = data.angular.x

        dataToWrite = '['+str(round(pitch_sp, 3))+';'+str(round(heading_sp, 3))+']'
        arduino.write(bytes(dataToWrite, 'utf-8'))
        #print(dataToWrite)
    


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('joystick')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        mcn = JoystickInputNode()
    except rospy.ROSInterruptException:
        pass
