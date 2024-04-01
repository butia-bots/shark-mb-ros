#!/usr/bin/env python3

import rospy
import bezier
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from generate_bezier_curve import generate_bezier_curve

class BezierShow():

    def __init__(self):
        rospy.init_node('show_bezier_curve')
        self.marker_pub = rospy.Publisher('bezier_curve_marker', Marker, queue_size=10)
        
        ctrl_pts = np.array([[-0.02780346, 0.03870414, 0.1697755, 0.2306312, 0.13624863, 0.04603491, -0.12468106, -0.20554966, -0.30316212, -0.30588269, -0.16874089, 0.01911827, 0.34452962, 0.48584107, 0.69099792, 1.03860146, 0.8180265, 0.81792381],
                             [-3.207116, -3.46167873, -4.03901895, -4.37807472, -4.96151693, -5.20991119, -5.66865206, -5.88474106, -6.41146359, -6.72947627, -7.43814356, -7.81679074, -8.60643786, -9.02458216, -9.79244157, -10.77088, -10.49220429, -10.49211049]])

        self.points = generate_bezier_curve(ctrl_pts)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.01
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    def run(self):
        for p in self.points:
            point = Point()
            point.x, point.y = p[0], p[1]
            self.marker.points.append(point)

        self.marker_pub.publish(self.marker)

def main(args=None):
    bezier_drawer = BezierShow()
    while True:
        bezier_drawer.run()

if __name__ == '__main__':
    main()

