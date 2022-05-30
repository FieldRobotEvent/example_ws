#!/usr/bin/env python3
import os
import csv

import rospy
import rospkg

from navigation import drive
from detection import detect

from std_msgs.msg import String


if __name__ == "__main__":
    rospy.init_node("task_mapping_node")

    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")

    # Read the relative position of the pillars
    markers_path = os.path.join(pkg_path, "map/markers.csv")
    with open(markers_path, "r") as f:
        reader = csv.reader(f)

        # Skip header
        next(reader)

        for row in reader:
            rospy.loginfo(
                f"Position {row[2]}: x={float(row[0]):.3f} y={float(row[1]):.3f}"
            )

    # Create publisher to notify evaluation
    weed_detection_publisher = rospy.Publisher("/fre_detections", String, queue_size=1)

    # Create path to file where we have to save the predicted coordinates of the
    # litter and weeds
    out_map_path = os.path.join(pkg_path, "map/pred_map.csv")

    try:
        # Open output file
        with open(out_map_path, "w") as f:
            rospy.loginfo("Writing map")
            writer = csv.writer(f)
            header = ["X", "Y", "kind"]
            writer.writerow(header)

            # Drive for 5 seconds
            rospy.loginfo("Starting to drive")

            for _ in range(5):
                drive(duration=1)

                # Detect something
                obj_type, obj_position = detect()
                rospy.loginfo(
                    f"Found {obj_type} at position x={obj_position[0]} y={obj_position[1]}"
                )

                # Notify evaluation that we have found something
                msg = String()
                msg.data = obj_type
                weed_detection_publisher.publish(msg)

                # Write found position to file
                writer.writerow([obj_position[0], obj_position[1], obj_type])
            
            rospy.loginfo("Done driving")

    except rospy.ROSInterruptException:
        pass
