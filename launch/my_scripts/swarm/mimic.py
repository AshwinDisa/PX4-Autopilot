#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr
import tf

class controller(): 

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.error = 0.1

        self.flag = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_state = State()

        rospy.Subscriber("uav1/mavros/state", State, self.state_callback)

        rospy.Subscriber('uav1/mavros/local_position/pose',
                            PoseStamped, self.position_callback)

    def control(self):

        rate = rospy.Rate(20)

        trajectory_mode.control()

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.pose.position.x + 3
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

class trajectory():

    def __init__(self, hover_position):

        self.drone_current_x = 0.0
        self.drone_current_y = 0.0
        self.drone_current_z = 0.0

        self.anti_drone_current_x = 0.0
        self.anti_drone_current_y = 0.0
        self.anti_drone_current_z = 0.0

        self.drone_previous_x = 0.0
        self.drone_previous_y = 0.0

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.time_to_impact = 0.7

        self.time_current = 0.0
        self.time_prev = 0.0

        self.desired_z = 6.0

        self.looping_time = 0.01            # 100Hz

        self.displacement = 0.0

        self.x_new = 0.0
        self.y_new = 0.0
        self.x_old = hover_position[0]
        self.y_old = hover_position[1]
        self.T_x = 0.0
        self.T_y = 0.0

        self.x_i_new = hover_position[0]
        self.y_i_new = hover_position[1]

        self.pose_publisher = rospy.Publisher('/pelican2/command/pose',
                                    PoseStamped, queue_size = 10)

        rospy.Subscriber('/pelican1/odometry_sensor1/pose',
                            Pose, self.drone_position_callback)

        rospy.Subscriber('/pelican2/odometry_sensor1/pose',
                            PoseStamped, self.anti_drone_position_callback)

    def control(self):

        # tranformation matrix

        quaternion = [self.x, self.y, self.z, self.w]
        yaw = self.quaternion_to_euler(quaternion)

        # print(yaw*180/math.pi)

        self.x_new = math.cos(yaw) * self.x_old - math.sin(yaw) * self.y_old + self.T_x
        self.y_new = math.sin(yaw) * self.x_old + math.cos(yaw) * self.y_old + self.T_y


        #self.x_old = self.x_new
        #self.y_old = self.y_new

        self.T_x = self.drone_current_x
        self.T_y = self.drone_current_y

        if (self.T_x > 0.5):

                print(self.T_x, self.T_y)

        self.drone_previous_x = self.drone_current_x
        self.drone_previous_y = self.drone_current_y

        #print(self.x_new, self.y_new)

        self.publisher(self.x_new, self.y_new)

    def quaternion_to_euler(self, quaternion):

        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return yaw

    def publisher(self, x_new, y_new):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = x_new - 3
        pose_msg.pose.position.y = y_new
        pose_msg.pose.position.z = 3.0

        self.pose_publisher.publish(pose_msg)

    def drone_position_callback(self, drone_data):

        self.drone_current_x = drone_data.position.x
        self.drone_current_y = drone_data.position.y
        self.drone_current_z = drone_data.position.z

        self.x = drone_data.orientation.x
        self.y = drone_data.orientation.y
        self.z = drone_data.orientation.z
        self.w = drone_data.orientation.w

    def anti_drone_position_callback(self, data):

        self.anti_drone_current_x = data.position.x + 3
        self.anti_drone_current_y = data.position.y
        self.anti_drone_current_z = data.position.z

if __name__ == "__main__":

    try:

        rospy.init_node("follower_offb_node_py")

        hover_position = np.array([3, 0, 3])
        trajectory_mode = trajectory(hover_position)

        drone_controller = controller(hover_position)
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
