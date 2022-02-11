#!/usr/bin/env python3
import os
import csv

import rospkg
import rospy
import std_msgs.msg

from navigation import driver
from detection import obj_mapper


def drive_and_publish(detection_publisher):
    # Drive for 3 seconds
    driver(3)

    # Report weed detection
    rospy.loginfo("report weed detection")
    detection_publisher.publish(std_msgs.msg.String("weed"))

    # Drive for 3 seconds
    driver(3)

    # Report litter detection
    rospy.loginfo("report litter detection")
    detection_publisher.publish(std_msgs.msg.String("litter"))

    # Drive for 3 seconds
    driver(3)


def write_prediction_map(weed_placements, litter_placements, out_map_path):
    # Save the prediction map
    with open(out_map_path, "w") as f:
        rospy.loginfo("writing map")
        writer = csv.writer(f)
        header = ["X", "Y", "kind"]
        writer.writerow(header)

        for elm in weed_placements:
            writer.writerow([elm[0], elm[1], "weed"])

        for elm in litter_placements:
            writer.writerow([elm[0], elm[1], "litter"])


def main():
    # Open the location marker file
    pkg_path = rospkg.RosPack().get_path("virtual_maize_field")
    marker_path = os.path.join(pkg_path, "map/markers.csv")

    with open(marker_path, "r") as markers_f:
        marker_reader = csv.reader(markers_f)

        rospy.loginfo("Markers loaded:")
        for line in marker_reader:
            rospy.loginfo("%s" % line)

    detection_publisher = rospy.Publisher(
        "fre_detections", std_msgs.msg.String, queue_size=10
    )

    # Drive through the field to detect weeds
    drive_and_publish(detection_publisher)

    # Detect weeds
    weed_placements, litter_placements = obj_mapper()

    # Write the prediction map
    out_map_path = os.path.join(pkg_path, "map/pred_map.csv")
    write_prediction_map(weed_placements, litter_placements, out_map_path)


if __name__ == "__main__":
    rospy.init_node("task_3_node")

    try:
        main()
    except rospy.ROSInterruptException:
        pass
