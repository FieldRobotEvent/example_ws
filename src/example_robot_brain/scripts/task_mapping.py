#!/usr/bin/env python3
import os
import csv

import rospy
import rospkg

from navigation import driver
from detection import obj_mapper

if __name__ == "__main__":
    rospy.init_node("task_mapping_node")

    # Read the driving directions from the file
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
    dp_path = os.path.join(pkg_path, "map/driving_pattern.txt")

    with open(dp_path) as direction_f:
        directions = direction_f.readline()

    rospy.loginfo(f"driving directions are: {directions}")

    try:
        # Drive for 5 seconds
        driver(5)

        weed_placements, litter_placements = obj_mapper()
        out_map_path = os.path.join(pkg_path, "map/pred_map.csv")
        with open(out_map_path, "w") as f:
            rospy.loginfo("writing map")
            writer = csv.writer(f)
            header = ["X", "Y", "kind"]
            writer.writerow(header)

            for elm in weed_placements:
                writer.writerow([elm[0], elm[1], "weed"])

            for elm in litter_placements:
                writer.writerow([elm[0], elm[1], "litter"])
        
    except rospy.ROSInterruptException:
        pass
