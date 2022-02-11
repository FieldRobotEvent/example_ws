#!/usr/bin/env python3
import os

import rospy
import rospkg

from navigation import driver

if __name__ == "__main__":
    rospy.init_node("task_2_node")

    # Read the driving idrections from the file
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
    dp_path = os.path.join(pkg_path, "map/driving_pattern.txt")

    with open(dp_path) as direction_f:
        directions = direction_f.readline()

    rospy.loginfo("driving directions are: %s" % directions)

    try:
        driver()
    except rospy.ROSInterruptException:
        pass
