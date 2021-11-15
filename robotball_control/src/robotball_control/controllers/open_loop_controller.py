#!/usr/bin/env python3
import math
import rospy

from geometry_msgs.msg import Twist, Vector3


class OpenLoopController(object):
    def __init__(self):
        self.start_auto = rospy.get_time()

        # Publishers
        self.cmd_vel = Twist()
        self.cmd_vel_pub = rospy.Publisher('open_vel', Twist, queue_size=1)

        # Subscribers

        # Open-loop control
        magnitudes = 0.5
        directions = [math.pi / 2, -math.pi / 2]
        times = [15, 15]
        segment = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - self.start_auto
            if elapsed > times[segment] and elapsed < times[segment] + 1:
                self.cmd_vel.linear = Vector3(0, directions[segment], 0)
            elif elapsed > times[segment] + 1:
                self.start_auto = rospy.get_time()
                segment = (segment + 1) % len(directions)
            else:
                self.cmd_vel.linear = Vector3(magnitudes, directions[segment], 0)

            self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=False)

    try:
        node = OpenLoopController()
    except rospy.ROSInterruptException:
        pass
