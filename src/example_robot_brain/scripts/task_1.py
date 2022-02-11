#!/usr/bin/env python3
import rospy

from navigation import driver

if __name__ == "__main__":
    rospy.init_node("task_1_node")

    try:
        driver()
    except rospy.ROSInterruptException:
        pass
