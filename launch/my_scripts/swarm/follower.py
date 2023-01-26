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

        rospy.wait_for_service("/uav1/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("uav1/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/uav1/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("uav1/mavros/set_mode", SetMode)

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown() and not self.current_state.connected):

            rate.sleep()

        while(not rospy.is_shutdown()):

            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):

                if(set_mode_client.call(offb_set_mode).mode_sent == True):

                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()

            else:

                if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):

                    if(arming_client.call(arm_cmd).success == True):

                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            displacement = math.sqrt(pow((self.hover_x - self.current_x),2) + pow((self.hover_y - self.current_y),2) + pow((self.hover_z - self.current_z),2))

            if (displacement > self.error and self.flag == 0):

                hover_mode.publisher()
                rate.sleep()

            else:

                self.flag = 1.0
                trajectory_mode.control()
                rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.pose.position.x + 3
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

class hover():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.pose_publisher = rospy.Publisher('uav1/mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = self.hover_x - 3
        pose_msg.pose.position.y = self.hover_y
        pose_msg.pose.position.z = self.hover_z

        self.pose_publisher.publish(pose_msg)

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


        self.vel_publisher = rospy.Publisher('uav1/mavros/setpoint_velocity/cmd_vel',
                                    TwistStamped, queue_size = 10)

        self.pose_publisher = rospy.Publisher('uav1/mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

        rospy.Subscriber('uav0/mavros/local_position/pose',
                            PoseStamped, self.drone_position_callback)

        rospy.Subscriber('uav1/mavros/local_position/pose',
                            PoseStamped, self.anti_drone_position_callback)

        rospy.Subscriber('/uav0/mavros/local_position/velocity_local',
                            TwistStamped, self.drone_velocity_callback)

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

        self.drone_current_x = drone_data.pose.position.x
        self.drone_current_y = drone_data.pose.position.y
        self.drone_current_z = drone_data.pose.position.z

        self.x = drone_data.pose.orientation.x
        self.y = drone_data.pose.orientation.y
        self.z = drone_data.pose.orientation.z
        self.w = drone_data.pose.orientation.w

    def anti_drone_position_callback(self, data):

        self.anti_drone_current_x = data.pose.position.x + 3
        self.anti_drone_current_y = data.pose.position.y
        self.anti_drone_current_z = data.pose.position.z

    def drone_velocity_callback(self, data):

        self.drone_current_vel_x = data.twist.linear.x
        self.drone_current_vel_y = data.twist.linear.y
        self.drone_current_vel_z = data.twist.linear.z

if __name__ == "__main__":

    try:

        rospy.init_node("follower_offb_node_py")

        hover_position = np.array([3, 0, 3])
        hover_mode = hover(hover_position)

        trajectory_mode = trajectory(hover_position)

        drone_controller = controller(hover_position)
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
