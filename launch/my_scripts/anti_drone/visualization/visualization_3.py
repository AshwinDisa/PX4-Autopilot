#!/usr/bin/env python3

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from geometry_msgs.msg import PoseStamped

class plot():

    def __init__(self):

        self.ax = plt.figure().gca(projection='3d')

        self.x = -13.85009479522705
        self.y = -16.234638214111328
        self.z = 9.992222785949707

        self.x_anti = 0.0
        self.y_anti = 0.0
        self.z_anti = 0.0

        rospy.Subscriber('/mavros/local_position/pose/old', PoseStamped, self.pose_callback)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.anti_drone_pose_callback)

    def pose_callback(self, pose_msg):

        self.x = pose_msg.pose.position.x
        self.y = pose_msg.pose.position.y
        self.z = pose_msg.pose.position.z

    def anti_drone_pose_callback(self, anti_pose_msg):

        self.x_anti = anti_pose_msg.pose.position.x
        self.y_anti = anti_pose_msg.pose.position.y
        self.z_anti = anti_pose_msg.pose.position.z

    def plotter(self):

        rate = rospy.Rate(20)

        ax = plt.axes(projection='3d')
        ax.set_xlim([-18, 5])
        ax.set_ylim([-18, 5])
        ax.set_zlim([0, 15])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Anti_drone system')

        s1 = []
        s2 = []
        s3 = []

        s4 = []
        s5 = []
        s6 = []

        while not rospy.is_shutdown():

            s1.append(self.x)
            s2.append(self.y)
            s3.append(self.z)

            s4.append(self.x_anti)
            s5.append(self.y_anti)
            s6.append(self.z_anti)

            #dot, = plt.plot(s1, s2, s3, 'r+')

            ax.plot(s1, s2, s3, 'red', label = 'drone trajectory')
            ax.plot(s4, s5, s6, 'green', label = 'anti_drone_trajectory')

            plt.pause(0.05)

            #dot.remove()

            rate.sleep()

if __name__ == '__main__':

    try:

        rospy.init_node("plotter")
        plot_obj = plot()
        plot_obj.plotter()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
