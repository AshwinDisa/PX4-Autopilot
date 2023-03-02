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

    def __init__(self):

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

        # main while loop

        while(not rospy.is_shutdown()):

            trajectory_mode.control()
            global time
            time += 1/self.looping_time
            rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

class trajectory():

    def __init__(self):

        self.drone_current_x = 0.0
        self.drone_current_y = 0.0
        self.drone_current_z = 0.0

        self.anti_drone_current_x = 0.0
        self.anti_drone_current_y = 0.0
        self.anti_drone_current_z = 0.0

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.drone_current_vel_x = 0.001
        self.drone_current_vel_y = 0.001
        self.drone_current_vel_z = 0.001

        self.looping_time = 0.05

        self.displacement = 0.0

        self.abs_weightage = 0.0
        self.rel_weightage = 0.0
        self.denominator = 0.0

        self.prev_vel_unit_sum = 0.0

        self.yaw_rate_P_gain = 0.02
        self.yaw_rate_D_gain = 0.01

        self.prev_yaw_error = 0.0

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

        heading_yaw = 180/math.pi*math.atan2((y_j - y_i),(x_j - x_i))

        # to estimate anti-drone current heading (yaw)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.anti_drone_quaternion)

        yaw_deg = 180 / math.pi * yaw

        # PD controller to control anti-drone heading towards drone at all times

        yaw_error = heading_yaw - yaw_deg

        drone_yaw_rate = self.yaw_rate_P_gain * (yaw_error) + self.yaw_rate_D_gain * (yaw_error - self.prev_yaw_error)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, heading_yaw * math.pi / 180)

        self.prev_yaw_error = yaw_error

        # to estimate heading between drone and anti-drone

        heading = self.drone_heading()

        # absolute velocity term, 3 times the current velocity of drone

        V = 3 * v_j * heading

        #displacement between drone and anti_drone

        displacement = math.sqrt(pow((self.anti_drone_current_x - self.drone_current_x),2) +
                                        pow((self.anti_drone_current_y - self.drone_current_y),2)
                                        + pow((self.anti_drone_current_z - self.drone_current_z),2))

        # to check if impact happened or not

        if (displacement < 1.0):

            # stop anti-drone and return to home

            self.auto_rtl()

        else:

            # to check if drone is travelling straight or changing direction

            drone_vel_vector = np.array([self.drone_current_vel_x, self.drone_current_vel_y,
                                                                    self.drone_current_vel_z])

            vel_change_diff = self.drone_path(drone_vel_vector)

            # estimate weightage of absolute and relative velocities

            abs_weightage = math.tanh((displacement + 10) / 50)
            rel_weightage = 1 - abs_weightage

            # estimate final velocity

            v_i_x = self.drone_current_vel_x * rel_weightage + V[0] * abs_weightage
            v_i_y = self.drone_current_vel_y * rel_weightage + V[1] * abs_weightage
            v_i_z = self.drone_current_vel_z * rel_weightage + V[2] * abs_weightage

            self.publisher(v_i_x, v_i_y, v_i_z, drone_yaw_rate, quaternion)

    def drone_heading(self):

        anti_drone_position_vector = np.array([- self.anti_drone_current_x + self.drone_current_x,
                                                - self.anti_drone_current_y + self.drone_current_y,
                                                - self.anti_drone_current_z + self.drone_current_z])

        magnitude_position_vector = self.magnitude(anti_drone_position_vector)

        return anti_drone_position_vector / magnitude_position_vector

    def drone_path(self, drone_vel_vector):

        vel_unit_vector = drone_vel_vector / self.magnitude(drone_vel_vector)

        vel_unit_sum = vel_unit_vector[0] + vel_unit_vector[1] + vel_unit_vector[2]

        vel_change_diff = vel_unit_sum - self.prev_vel_unit_sum

        self.prev_vel_unit_sum = vel_unit_sum

        return vel_change_diff

    def auto_rtl(self):

        v_i_x = 0.0
        v_i_y = 0.0
        v_i_z = 0.0

        vel_msg = TwistStamped()

        vel_msg.twist.linear.x = v_i_x
        vel_msg.twist.linear.y = v_i_y
        vel_msg.twist.linear.z = v_i_z

        self.vel_publisher.publish(vel_msg)

        print("THROW NET")

        rospy.sleep(1)

        # service request for Return to Home

        # offb_set_mode = SetModeRequest()
        # offb_set_mode.custom_mode = 'AUTO.RTL'

        # if (self.current_state.mode != "AUTO.RTL"):

        #     if (self.set_mode_client.call(offb_set_mode).mode_sent == True):

        #         rospy.loginfo("RTL enabled")
        #         rospy.signal_shutdown("impact")

        rospy.signal_shutdown("impact")

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def magnitude(self, vector):

        return math.sqrt(sum(pow(element, 2) for element in vector))

    def publisher(self, v_i_x, v_i_y, v_i_z, v_i_yaw, quaternion):

        # limit velocities in all directions

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

        # publish velocity message

        vel_msg = TwistStamped()

        vel_msg.twist.linear.x = v_i_x
        vel_msg.twist.linear.y = v_i_y
        vel_msg.twist.linear.z = v_i_z

        vel_msg.twist.angular.z = v_i_yaw

        self.vel_publisher.publish(vel_msg)

    def drone_position_callback(self, drone_data):

        self.drone_current_x = drone_data.pose.position.x
        self.drone_current_y = drone_data.pose.position.y
        self.drone_current_z = drone_data.pose.position.z

    def anti_drone_position_callback(self, data):

        self.anti_drone_current_x = data.pose.position.x
        self.anti_drone_current_y = data.pose.position.y
        self.anti_drone_current_z = data.pose.position.z

        self.anti_drone_quat_x = data.pose.orientation.x
        self.anti_drone_quat_y = data.pose.orientation.y
        self.anti_drone_quat_z = data.pose.orientation.z
        self.anti_drone_quat_w = data.pose.orientation.w

        self.anti_drone_quaternion = np.array([self.anti_drone_quat_x, self.anti_drone_quat_y,
                                            self.anti_drone_quat_z, self.anti_drone_quat_w])

    def drone_velocity_callback(self, data):

        self.drone_current_vel_x = data.twist.linear.x
        self.drone_current_vel_y = data.twist.linear.y
        self.drone_current_vel_z = data.twist.linear.z

if __name__ == "__main__":

    try:

        rospy.init_node("anti_drone_method_2")

        # initialize trajectory object

        trajectory_mode = trajectory()

        # initialize main control object and call control function

        drone_controller = controller()
        drone_controller.control()

    except rospy.ROSInterruptException:

        print("Total time = ", time)
        pass

