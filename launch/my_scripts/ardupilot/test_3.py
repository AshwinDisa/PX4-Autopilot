#! /usr/bin/env python3

"""
code to test velocity input
Input cmd_vel = [0.5, 0, 0]
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr

class controller():

    def __init__(self):

        self.flag = 0.0

        self.current_state = State()

        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def control(self):

        rate = rospy.Rate(20)

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown() and not self.current_state.connected):

            rate.sleep()

        while(not rospy.is_shutdown()):

            vel_pub_mode.publisher()
            rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

class vel_pub():

    def __init__(self, cmd_vel):

        self.vel_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                    TwistStamped, queue_size = 10)

    def publisher(self):

        vel_msg = TwistStamped()

        vel_msg.twist.linear.x = cmd_vel[0]
        vel_msg.twist.linear.y = cmd_vel[1]
        vel_msg.twist.linear.z = cmd_vel[2]

        self.vel_publisher.publish(vel_msg)

if __name__ == "__main__":

    try:

        rospy.init_node("drone_offb_node_py")

        cmd_vel = np.array([0.5, 0, 0])

        vel_pub_mode = vel_pub(cmd_vel)

        drone_controller = controller()
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
