#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Broadcast tf data during simulation.

tf data is produced from position of each robot received on Odometry messages.
It is used to visualize simulated robots in Rviz.
"""

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


class SimTF(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        num_of_robots = rospy.get_param("/num_of_robots")
        [rospy.Subscriber("/robot_{}/odom".format(i), Odometry, self.callback) for i in range(num_of_robots)]

        rospy.spin()

    def callback(self, msg):
        try:
            tf = geometry_msgs.msg.TransformStamped()
            tf.child_frame_id = msg.header.frame_id
            tf.header.frame_id = "world"
            tf.header.stamp = msg.header.stamp

            tf.transform.translation.x = 0
            tf.transform.translation.y = 0
            tf.transform.translation.z = 0

            tf.transform.rotation.x = 0
            tf.transform.rotation.y = 0
            tf.transform.rotation.z = 0
            tf.transform.rotation.w = 1

            self.broadcaster.sendTransform(tf)

        except BaseException as exc:
            print(exc)
            self.tfBuffer.clear()
            return


if __name__ == '__main__':
    rospy.init_node('simulation_tf')

    try:
        node = SimTF()
    except rospy.ROSInterruptException:
        pass
