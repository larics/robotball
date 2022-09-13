#!/usr/bin/env python3

import rospy


from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from dynamic_reconfigure import DynamicReconfigureCallbackException
from robotball_driver.cfg import GlobalDriverConfig


class GlobalDriverReconf(object):
    def __init__(self):
        # Dynamic reconfigure server for PID tuning
        self.clients = {f'robot_{i}': Client(f'/robot_{i}/driver') for i in range(1, 4)}
        self.server = Server(GlobalDriverConfig, self.reconfigure_callback)

        rospy.spin()

    def reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request:
            robot_1: {robot_1}
            robot_2: {robot_2}
            robot_3: {robot_3}
            offset: {offset}
            """.format(**config))

        for name, client in self.clients.items():
            try:
                client.update_configuration({"joystick": config[f'{name}'],
                                            "offset": config['offset']})
            except DynamicReconfigureCallbackException as e:
                rospy.logerr(f"DynReconf service call for {name} failed.")

        return config


if __name__ == "__main__":
    rospy.init_node("driver", anonymous=False)

    try:
        node = GlobalDriverReconf()
    except rospy.ROSInterruptException:
        pass
