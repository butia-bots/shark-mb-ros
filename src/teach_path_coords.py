#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from save_coords_to_file import save_coords_to_file

class TeachPathCoords:

    def __init__(self):
        rospy.init_node('teach_path_coords')

        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        self.teleop_points = []
        self.file_path = '/home/fbotathome/fbot_ws/src/shark-mb-ros/data/teleop_data.txt'

        print("Recording... Press CTRL + C to save coords.")

    def callback(self, msg : PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = Point()
        point.x = x
        point.y = y
        self.teleop_points.append(point)

def main():
    try:
        teach_path_coords = TeachPathCoords()
        rospy.spin()
    finally:
        save_coords_to_file(teach_path_coords.file_path, teach_path_coords.teleop_points)

if __name__ == '__main__':
    main()
