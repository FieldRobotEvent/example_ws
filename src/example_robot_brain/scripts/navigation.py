#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist


def drive(duration=10):
    # A very simple script that just sends the robot forwards for n seconds. We expect that you can come up with a
    # better navigation algorithm.
    pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    for _ in range(duration * 10):
        msg = Twist()
        msg.linear.x = 1
        pub.publish(msg)
        rate.sleep()

