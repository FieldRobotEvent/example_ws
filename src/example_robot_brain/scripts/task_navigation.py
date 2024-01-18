#!/usr/bin/env python3

import rospy

from navigation import drive

from virtual_maize_field import get_driving_pattern_file


if __name__ == "__main__":
    rospy.init_node("task_navigation_node")

    with open(get_driving_pattern_file()) as direction_f:
        directions = direction_f.readline()

    rospy.loginfo(f"Driving directions are: {directions}")

    try:
        # Drive
        rospy.loginfo("Starting to drive")

        drive()

        rospy.loginfo("Done driving")
    except rospy.ROSInterruptException:
        pass
