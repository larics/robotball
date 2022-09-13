#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from robotball_control.cfg import GlobalBilliardConfig


class GlobalBilliardReconf(object):
    def __init__(self):
        # Dynamic reconfigure server for PID tuning
        self.clients = [Client(f'/robot_{i}/billiard_controller', timeout=2.0) for i in range(1, 4)]
        self.server = Server(GlobalBilliardConfig, self.reconfigure_callback)

        rospy.spin()

    def reconfigure_callback(self, config, level):
        rospy.loginfo("Paramaters updated!")

        for client in self.clients:
            client.update_configuration(config)

        return config


if __name__ == "__main__":
    rospy.init_node("global_billiard_reconf", anonymous=False)

    try:
        node = GlobalBilliardReconf()
    except rospy.ROSInterruptException:
        pass
