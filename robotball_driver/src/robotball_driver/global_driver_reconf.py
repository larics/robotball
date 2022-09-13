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
from robotball_driver.cfg import GlobalDriverConfig


class GlobalDriverReconf(object):
    def __init__(self):
        self.first_pass = True
        self.mode_manual = True
        self.latest_config = None
        self.enabled = False

        self.position = Vector3()
        self.last_direction = 0

        rospy.sleep(1)

        # Dynamic reconfigure server for PID tuning
        self.clients = [Client(f'/robot_{i}/driver', timeout=2.0) for i in range(1, 4)]
        self.server = Server(GlobalDriverConfig, self.reconfigure_callback)

        rospy.spin()

    def reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request:
            robot_1: {robot_1}
            robot_2: {robot_2}
            robot_3: {robot_3}
            offset: {offset}
            """.format(**config))

        for i, client in enumerate(self.clients, start=1):
            client.update_configuration({"joystick": config[f'robot_{i}'],
                                         "offset": config['offset']})

        return config


if __name__ == "__main__":
    rospy.init_node("driver", anonymous=False)

    try:
        node = GlobalDriverReconf()
    except rospy.ROSInterruptException:
        pass
