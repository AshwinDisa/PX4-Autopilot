#!/usr/bin/env python3
import rospy
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import time
import tf.transformations
import math
import modern_robotics as mr

class controller():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.error = 0.1

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        rospy.Subscriber('/uav0/mavros/local_position/pose',
                            PoseStamped, self.state_callback)

    def control(self):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            displacement = math.sqrt(pow((self.hover_x - self.current_x),2) +
                                        pow((self.hover_y - self.current_y),2)
                                        + pow((self.hover_z - self.current_z),2))

            if (displacement > self.error):

                hover_mode.publisher()
                rate.sleep()

            else:

                trajectory_mode.control()


    def state_callback(self, data):

        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

class hover():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.pose_publisher = rospy.Publisher('/uav1/mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.hover_x
        pose_msg.pose.position.y = self.hover_y
        pose_msg.pose.position.z = self.hover_z

        self.pose_publisher.publish(pose_msg)

class trajectory():

    def __init__(self, traj_velocity):

        self.velocity_x = traj_velocity[0]
        self.velocity_y = traj_velocity[1]
        self.velocity_z = traj_velocity[2]

        self.vel_publisher = rospy.Publisher('/uav0/mavros/setpoint_velocity/cmd_vel',
                                        TwistStamped, queue_size = 10)


    def control(self):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            trajectory_mode.publisher()
            rate.sleep()

    def publisher(self):

        vel_msg = Twist()

        vel_msg.twist.linear.x = self.velocity_x
        vel_msg.twist.linear.y = self.velocity_y
        vel_msg.twist.linear.z = self.velocity_z

        self.vel_publisher.publish(vel_msg)


if __name__ == '__main__':

    try:

        rospy.init_node('drone_node')

        hover_position = [5,0,3]
        traj_velocity = [5,0,0]

        traj_initial_pos = np.array(hover_position)
        traj_final_pos = np. array([-2, 0, 3])

        Xstart = np.zeros((4, 4))
        Xend = np.zeros((4, 4))

        Xstart[:3, 3] = traj_initial_pos
        Xend[:3, 3] = traj_final_pos

        Tf = 25                             # time to reach from start to end position
        samplingTime = 1/100                # sampling time in seconds
        N = int(Tf/samplingTime)            # number of samples
        method = 5                          # interpolation method
        trajectory = mr.CartesianTrajectory(
                Xstart, Xend, Tf, N, method)    # get the trajectory
        timegap = Tf / (N - 1.0)

        # print(len(trajectory))

        hover_mode = hover(hover_position)

        trajectory_mode = trajectory(traj_velocity)

        drone_controller = controller(hover_position)
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
