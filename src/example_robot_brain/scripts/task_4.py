#!/usr/bin/env python3
import os
import csv

import rospkg
import rospy

from navigation import driver

if __name__ == "__main__":
    rospy.init_node("task_3_node")

    # Open the provided map
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
    map_path = os.path.join(pkg_path, "map/map.csv")
    with open(map_path) as map_f:
        map_reader = csv.reader(map_f)

        rospy.loginfo("map loaded")
        for line in map_reader:
            rospy.loginfo("%s" % line)

    # Drive for 10 seconds
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
