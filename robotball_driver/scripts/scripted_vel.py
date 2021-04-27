#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Twist

# Set parameters here
v_ref       = 1     # Desired total velocity (all tests)
drive_time  = 4     # During this time, desired velocity is commanded. (tests 1,2,4)
stop_time   = 1     # Time it takes the robot to stop after zero velocity is sent. (tests 1,2,4)
r_ref       = 2.5   # Desired radius (test 3)
max_time    = 20    # Duration in seconds (test 3)
max_iter    = 3     # Number of repetitions (test 4)
active_test = 1     # Index of the test


def test_1(cmd_vel, elapsed_time):
    if elapsed_time < drive_time:
        cmd_vel.linear.x = v_ref
        cmd_vel.linear.y = 0
        return False
    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        return True

def test_2(cmd_vel, elapsed_time):
    if elapsed_time < drive_time:
        cmd_vel.linear.x = math.sqrt(2)/2 * v_ref
        cmd_vel.linear.y = math.sqrt(2)/2 * v_ref
    elif elapsed_time < drive_time + stop_time:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
    elif elapsed_time < 2 * drive_time + stop_time:
        cmd_vel.linear.x = -math.sqrt(2)/2 * v_ref
        cmd_vel.linear.y = -math.sqrt(2)/2 * v_ref
    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        return True
    return False

def test_3(cmd_vel, elapsed_time):
    if elapsed_time > max_time:
        return True

    w = v_ref / r_ref
    cmd_vel.linear.x = math.sin(elapsed_time * w)
    cmd_vel.linear.y = math.cos(elapsed_time * w)
    return False


def test_4(cmd_vel, elapsed_time):
    iteration_time = elapsed_time % (2 * (drive_time + stop_time))
    iteration_num = elapsed_time // (2 * (drive_time + stop_time))


    cmd_vel.linear.x = 0
    cmd_vel.linear.y = 0

    if iteration_num < max_iter:
        if iteration_time < drive_time:
            cmd_vel.linear.x = v_ref
            cmd_vel.linear.y = 0
        elif iteration_time < drive_time + stop_time:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
        elif iteration_time < 2 * drive_time + stop_time:
            cmd_vel.linear.x = -1
            cmd_vel.linear.y = -0
    else:
        return True
    return False


if __name__ == "__main__":
    rospy.init_node("commander")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    cmd_vel = Twist()

    rospy.sleep(2)

    # Infinite loop which sends whatever is currently in cmd_vel.
    rate = rospy.Rate(10)
    start = rospy.get_time()
    test_complete = False
    while not rospy.is_shutdown() and not test_complete:
        elapsed_time = rospy.get_time() - start

        # Calculate cmd_vel necessary for the test.
        if active_test == 1:
            test_complete = test_1(cmd_vel, elapsed_time)
        if active_test == 2:
            test_complete = test_2(cmd_vel, elapsed_time)
        if active_test == 3:
            test_complete = test_3(cmd_vel, elapsed_time)
        if active_test == 4:
            test_complete = test_4(cmd_vel, elapsed_time)

        pub.publish(cmd_vel)

        rate.sleep()

    rospy.logwarn("Test completed.")
