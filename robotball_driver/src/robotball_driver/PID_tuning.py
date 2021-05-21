#!/usr/bin/env python

import rospy
import json

from dynamic_reconfigure.server import Server
from robotball_driver.cfg import PIDConfig

class PIDtuning(object):
    def __init__(self):

        self.PID_types = ['vel', 'pitch', 'hdg']
        self.PID_values = {key: {} for key in self.PID_types}
        self.srv = Server(PIDConfig, self.callback)

        rospy.spin()

    def callback(self, config, level):
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
        for t in self.PID_types:
            for v in ['P', 'I', 'D']:
                self.PID_values[t][v] = config['{}_{}'.format(t, v)]

        return config

if __name__ == "__main__":
    rospy.init_node("PID_tuning", anonymous = False)

    try:
        node = PIDtuning()
    except rospy.ROSInterruptException:
        pass