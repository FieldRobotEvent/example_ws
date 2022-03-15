#!/usr/bin/env python3
import os

import rospy
import rospkg

from navigation import driver

if __name__ == "__main__":
    rospy.init_node("task_navigation_node")

    # Read the driving directions from the file
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
    dp_path = os.path.join(pkg_path, "map/driving_pattern.txt")

    with open(dp_path) as direction_f:
        directions = direction_f.readline()

    rospy.loginfo(f"Driving directions are: {directions}")

    try:
        # Drive for 5 seconds
        driver(5)
    except rospy.ROSInterruptException:
        pass
