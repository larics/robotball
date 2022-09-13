#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from dynamic_reconfigure import DynamicReconfigureCallbackException
from robotball_control.cfg import GlobalBilliardConfig


class GlobalBilliardReconf(object):
    def __init__(self):
        # Dynamic reconfigure server for PID tuning
        self.clients = {f'robot_{i}': Client(f'/robot_{i}/billiard_controller') for i in range(1, 4)}
        self.server = Server(GlobalBilliardConfig, self.reconfigure_callback)

        rospy.spin()

    def reconfigure_callback(self, config, level):
        rospy.loginfo("Parameters updated!")

        for name, client in self.clients.items():
            try:
                client.update_configuration(config)
            except DynamicReconfigureCallbackException as e:
                rospy.logerr(f"DynReconf service call for {name} failed.")

        return config


if __name__ == "__main__":
    rospy.init_node("global_billiard_reconf", anonymous=False)

    try:
        node = GlobalBilliardReconf()
    except rospy.ROSInterruptException:
        pass
