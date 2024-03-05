#!/usr/bin/env python3

import time
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class Navigator:
    THRESHOLD_YAW = .5
    THRESHOLD_DIST = .5

    def __init__(self):    
        self.cmd_vel_pub = rospy.Publisher('/hoverboard_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.position_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback, queue_size=1)

        self.read_points_from_file('/home/fbotathome/fbot_ws/src/shark-mb-ros/data/teleop_data.txt')

    def read_points_from_file(self, file_path):
        self.goals = np.empty((0, 2))
        with open(file_path, 'r') as f:
            for line in f:
                x, y = line.strip().split(',')
                point = np.array([[float(x), float(y)]])
                self.goals = np.append(self.goals, point, axis = 0)
        return

    def odom_callback(self, data: PoseWithCovarianceStamped):
        msg = Twist()

        if len(self.goals) == 0:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            return

        quaternion = data.pose.pose.orientation 
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.yaw = euler_from_quaternion(quaternion_list)

        print("self.yaw: ", self.yaw)

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.pos = np.array([self.x, self.y])

        goal = self.goals[0]
        goal_vec = np.array(goal)
        self.dist_vec = goal_vec - self.pos
        self.dist = np.linalg.norm(self.dist_vec)
        print("self.dist: ", self.dist)

        self.yaw_d = np.arctan2(self.dist_vec[1], self.dist_vec[0])
        self.diff_yaw = self.yaw_d - self.yaw

        print("Goals", self.goals)
    
        if self.diff_yaw > self.THRESHOLD_YAW:
            msg.linear.x = .0
            msg.angular.z = .1
            print("cond 1")
        elif self.diff_yaw < -self.THRESHOLD_YAW:
            msg.linear.x = .0
            msg.angular.z = -.1
            print("cond 2")
        else:
            msg.linear.x = .5
            msg.angular.z = .0
            print("cond 3")

        self.cmd_vel_pub.publish(msg)
        print("msg: ", msg)

        print("velocidades setadas")

        if self.dist < self.THRESHOLD_/hoverboard_velocity_controller/cmd_vel
DIST:
            rospy.loginfo("Atingiu o objetivo")
            self.goals = np.delete(self.goals, 0, 0)
            self.cmd_vel_pub.publish(msg)
            rospy.loginfo(self.goals)
            if len(self.goals) == 0:
                rospy.loginfo("Objetivos concluídos")
                msg.angular.z = .0
                msg.linear.x = .0
        
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('bug1')
    Navigator()
    rospy.spin()