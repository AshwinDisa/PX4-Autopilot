#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr
import tf

global time
time = 0.0

class controller():

    def __init__(self, hover_position, spawn_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.error = 0.3

        self.flag = 0.0

        self.looping_time = 0.05

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_state = State()

        rospy.Subscriber("/mavros/state", State, self.state_callback)

        rospy.Subscriber('/mavros/local_position/pose',
                            PoseStamped, self.position_callback)

    def control(self):

        rate = rospy.Rate(1/self.looping_time)

        rospy.sleep(1)

        while(not rospy.is_shutdown()):

            trajectory_mode.control()
            global time
            time += 1/self.looping_time
            rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.pose.position.x + spawn_position[0]
        self.current_y = data.pose.position.y + spawn_position[1]
        self.current_z = data.pose.position.z + spawn_position[2]

class hover():

    def __init__(self, hover_position, spawn_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = self.hover_x - spawn_position[0]
        pose_msg.pose.position.y = self.hover_y - spawn_position[1]
        pose_msg.pose.position.z = self.hover_z - spawn_position[2]

        self.pose_publisher.publish(pose_msg)

class trajectory():

    def __init__(self, hover_position, spawn_position, spawn_position_drone):

        self.drone_current_x = 0.0
        self.drone_current_y = 0.0
        self.drone_current_z = 0.0

        self.anti_drone_current_x = 0.0
        self.anti_drone_current_y = 0.0
        self.anti_drone_current_z = 0.0

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.time_current = 0.0
        self.time_prev = 0.0

        self.desired_z = 6.0

        self.looping_time = 0.05          # 20Hz

        self.displacement = 0.0

        self.abs_weightage = 0.0
        self.rel_weightage = 0.0
        self.denominator = 0.0

        self.flag = 0.0

        self.prev_vel_unit_sum = 0.0

        self.yaw_rate_gain = 0.01

        self.quaternion = np.array([0, 0, 0, 0])

        self.vel_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                    TwistStamped, queue_size = 10)

        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

        rospy.Subscriber('/mavros/local_position/pose/old',
                            PoseStamped, self.drone_position_callback)

        rospy.Subscriber('/mavros/local_position/pose',
                            PoseStamped, self.anti_drone_position_callback)

        rospy.Subscriber('mavros/local_position/velocity_local/old',
                            TwistStamped, self.drone_velocity_callback)

        rospy.Subscriber("/mavros/state", State, self.state_callback)

        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def control(self):

        x_j = self.drone_current_x
        x_i = self.anti_drone_current_x
        y_j = self.drone_current_y
        y_i = self.anti_drone_current_y
        v_j = math.sqrt(pow((self.drone_current_vel_x),2) +
                                        pow((self.drone_current_vel_y),2)
                                        + pow((self.drone_current_vel_z),2))


        self.drone_position_vector = np.array([self.drone_current_x, self.drone_current_y, self.drone_current_z])

        self.anti_drone_position_vector = np.array([- self.anti_drone_current_x + self.drone_current_x,
                                                - self.anti_drone_current_y + self.drone_current_y,
                                                - self.anti_drone_current_z + self.drone_current_z])

        self.magnitude_position_vector = self.magnitude(self.anti_drone_position_vector)

        heading = self.anti_drone_position_vector / self.magnitude_position_vector

        heading_yaw = 180/math.pi*math.atan2((y_j - y_i),(x_j - x_i))

        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(self.anti_drone_quaternion)

        yaw_deg = 180/math.pi*self.yaw

        yaw_error = heading_yaw - yaw_deg

        print(yaw_error)

        self.drone_yaw_rate = self.yaw_rate_gain * (yaw_error)

        self.quaternion = tf.transformations.quaternion_from_euler(0, 0, heading_yaw*math.pi/180)

        V = 3 * v_j * heading

        displacement = math.sqrt(pow((self.anti_drone_current_x - self.drone_current_x),2) +
                                        pow((self.anti_drone_current_y - self.drone_current_y),2)
                                        + pow((self.anti_drone_current_z - self.drone_current_z),2))

        if (displacement < 1.0):

            self.v_i_x = 0.0
            self.v_i_y = 0.0
            self.v_i_z = 0.0

            vel_msg = TwistStamped()

            vel_msg.twist.linear.x = self.v_i_x
            vel_msg.twist.linear.y = self.v_i_y
            vel_msg.twist.linear.z = self.v_i_z

            self.vel_publisher.publish(vel_msg)

            print("THROW NET")

            rospy.sleep(1)

            # offb_set_mode = SetModeRequest()
            # offb_set_mode.custom_mode = 'AUTO.RTL'

            # if (self.current_state.mode != "AUTO.RTL"):

            #     if (self.set_mode_client.call(offb_set_mode).mode_sent == True):

            #         rospy.loginfo("RTL enabled")
            #         rospy.signal_shutdown("impact")

            rospy.signal_shutdown("impact")

        else:

            self.drone_vel_vector = np.array([self.drone_current_vel_x, self.drone_current_vel_y, self.drone_current_vel_z])
            vel_unit_vector = self.drone_vel_vector / self.magnitude(self.drone_vel_vector)
            self.vel_unit_sum = vel_unit_vector[0] + vel_unit_vector[1] + vel_unit_vector[2]
            vel_change_diff = self.vel_unit_sum - self.prev_vel_unit_sum

            if (vel_change_diff > 0.005 or vel_change_diff < -0.005 or vel_change_diff == 0.0):

                self.denominator = 40

            else:

                self.denominator = -4500000000000000 * math.pow(vel_change_diff, 6) + 110

            self.prev_vel_unit_sum = self.vel_unit_sum

            # print(vel_change_diff, self.denominator)

            self.abs_weightage = math.tanh((displacement + 10) / self.denominator)
            self.rel_weightage = 1 - self.abs_weightage

            self.v_i_x = self.drone_current_vel_x * self.rel_weightage + V[0] * self.abs_weightage
            self.v_i_y = self.drone_current_vel_y * self.rel_weightage + V[1] * self.abs_weightage
            self.v_i_z = self.drone_current_vel_z * self.rel_weightage + V[2] * self.abs_weightage

            self.publisher(self.v_i_x, self.v_i_y, self.v_i_z, self.drone_yaw_rate, self.quaternion)

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def magnitude(self, vector):

        return math.sqrt(sum(pow(element, 2) for element in vector))

    def publisher(self, v_i_x, v_i_y, v_i_z, v_i_yaw, quaternion):

        # if (v_i_x > 4.0):

        #     v_i_x = 3.0
        #     print("limiting vel")

        # if (v_i_x < -4.0):

        #     v_i_x = -3.0
        #     print("limiting vel")

        # if (v_i_y > 4.0):

        #     v_i_y = 3.0
        #     print("limiting vel")

        # if (v_i_y < -4.0):

        #     v_i_y = -3.0
        #     print("limiting vel")

        # if (v_i_z > 4.0):

        #     v_i_z = 3.0
        #     print("limiting vel")

        # if (v_i_z < -4.0):

        #     v_i_z = -3.0
        #     print("limiting vel")

        if (self.flag == 0.0):

            self.x_i_old = self.anti_drone_current_x
            self.y_i_old = self.anti_drone_current_y
            self.z_i_old = self.anti_drone_current_z

            self.flag = 1.0

        self.x_i_new = self.x_i_old + self.v_i_x * self.looping_time
        self.y_i_new = self.y_i_old + self.v_i_y * self.looping_time
        self.z_i_new = self.z_i_old + self.v_i_z * self.looping_time

        # print(self.x_i_new, self.y_i_new)

        pose_msg = PoseStamped()
        vel_msg = TwistStamped()

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pose_msg.pose.position.x = self.x_i_new
        pose_msg.pose.position.y = self.y_i_new
        pose_msg.pose.position.z = self.z_i_new

        vel_msg.twist.linear.x = v_i_x
        vel_msg.twist.linear.y = v_i_y
        vel_msg.twist.linear.z = v_i_z

        vel_msg.twist.angular.z = v_i_yaw

        self.vel_publisher.publish(vel_msg)

        # self.pose_publisher.publish(pose_msg)

        # self.vel_publisher.publish(vel_msg)

        self.x_i_old = self.x_i_new
        self.y_i_old = self.y_i_new
        self.z_i_old = self.z_i_new

    def drone_position_callback(self, drone_data):

        self.drone_current_x = drone_data.pose.position.x + spawn_position_drone[0]
        self.drone_current_y = drone_data.pose.position.y + spawn_position_drone[1]
        self.drone_current_z = drone_data.pose.position.z + spawn_position_drone[2]

    def anti_drone_position_callback(self, data):

        self.anti_drone_current_x = data.pose.position.x + spawn_position[0]
        self.anti_drone_current_y = data.pose.position.y + spawn_position[1]
        self.anti_drone_current_z = data.pose.position.z + spawn_position[2]

        self.anti_drone_quat_x = data.pose.orientation.x
        self.anti_drone_quat_y = data.pose.orientation.y
        self.anti_drone_quat_z = data.pose.orientation.z
        self.anti_drone_quat_w = data.pose.orientation.w

        self.anti_drone_quaternion = np.array([self.anti_drone_quat_x, self.anti_drone_quat_y, self.anti_drone_quat_z, self.anti_drone_quat_w])

    def drone_velocity_callback(self, data):

        self.drone_current_vel_x = data.twist.linear.x
        self.drone_current_vel_y = data.twist.linear.y
        self.drone_current_vel_z = data.twist.linear.z


if __name__ == "__main__":

    try:

        rospy.init_node("anti_drone_method_2")
        hover_position = [0, 0, 2]
        spawn_position = [0 ,0, 0]
        spawn_position_drone = [0, 0, -0.1]

        hover_mode = hover(hover_position, spawn_position)
        trajectory_mode = trajectory(hover_position, spawn_position, spawn_position_drone)

        drone_controller = controller(hover_position, spawn_position)
        drone_controller.control()

    except rospy.ROSInterruptException:

        print("Total time = ", time)
        pass

