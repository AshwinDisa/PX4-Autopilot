#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr

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

            displacement = math.sqrt(pow((self.hover_x - self.current_x),2) +
                                        pow((self.hover_y - self.current_y),2)
                                        + pow((self.hover_z - self.current_z),2))

            if (displacement > self.error and self.flag == 0.0):

                hover_mode.publisher()
                rate.sleep()

            else:

                self.flag = 1.0
                trajectory_mode.control()

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

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.time_to_impact = 0.7

        self.time_current = 0.0
        self.time_prev = 0.0

        self.desired_z = 6.0

        self.looping_time = 0.01            # 100Hz

        self.displacement = 0.0

        self.x_i_new = hover_position[0]
        self.y_i_new = hover_position[1]


        self.vel_publisher = rospy.Publisher('uav1/mavros/setpoint_velocity/cmd_vel',
                                    TwistStamped, queue_size = 10)

        rospy.Subscriber('uav0/mavros/local_position/pose',
                            PoseStamped, self.drone_position_callback)

        rospy.Subscriber('uav1/mavros/local_position/pose',
                            PoseStamped, self.anti_drone_position_callback)

        rospy.Subscriber('/uav0/mavros/local_position/velocity_local',
                            TwistStamped, self.drone_velocity_callback)

    def control(self):

        x_j = self.drone_current_x
        x_i = self.anti_drone_current_x
        y_j = self.drone_current_y
        y_i = self.anti_drone_current_y
        v_j = math.sqrt(pow((self.drone_current_vel_x),2) +
                                        pow((self.drone_current_vel_y),2)
                                        + pow((self.drone_current_vel_z),2))

        phi = math.atan(y_j/x_j)

        self.displacement = math.sqrt(pow((self.anti_drone_current_x - self.drone_current_x),2) +
                                pow((self.anti_drone_current_y - self.drone_current_y),2)
                                + pow((self.anti_drone_current_z - self.drone_current_z),2))

        if (self.displacement < 2.5):

                self.time_to_impact = 0.3

        else:

                self.time_to_impact = 0.7

        v_i_cos_theta = (x_j - x_i + v_j * math.cos(phi) * self.time_to_impact) / self.time_to_impact
        v_i_sin_theta = (y_j - y_i + v_j * math.sin(phi) * self.time_to_impact) / self.time_to_impact

        # print(self.displacement, self.time_to_impact)

        theta = math.atan(v_i_sin_theta / v_i_cos_theta)

        v_i = v_i_sin_theta / math.sin(theta)

        # print(theta*180/math.pi, v_i)
        # print(v_j)

        self.v_i_x = v_i * math.cos(theta)
        self.v_i_y = v_i * math.sin(theta)

        self.x_i_new = self.x_i_new + self.v_i_x * self.looping_time
        self.y_i_new = self.y_i_new + self.v_i_y * self.looping_time

        # print(v_i, x_i_new, y_i_new)
        # print("time ", self.time_current - self.time_prev)
        # print("x_i, y_i ", self.x_i_new, self.y_i_new)


        self.publisher(self.v_i_x, self.v_i_y)

    def publisher(self, v_i_x, v_i_y):

        vel_msg = TwistStamped()

        vel_msg.twist.linear.x = v_i_x
        vel_msg.twist.linear.y = v_i_y

        self.vel_publisher.publish(vel_msg)

    def drone_position_callback(self, drone_data):

        self.drone_current_x = drone_data.pose.position.x
        self.drone_current_y = drone_data.pose.position.y
        self.drone_current_z = drone_data.pose.position.z

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

        rospy.init_node("anti_drone_offb_node_py")
        hover_position = [0, 0, 5]

        hover_mode = hover(hover_position)
        trajectory_mode = trajectory(hover_position)

        drone_controller = controller(hover_position)
        drone_controller.control()

    except rospy.ROSInterruptException:

        pass
