#! /usr/bin/env python3

"""
Gives pose commands in x, y, z
Input: hover position = [10, 0, 5]

Takeoff with rc or qgc.
Stay in Position mode.
Run script.
Change to guided mode manually.
If position mode is set by the auto-pilot say after termination of script, it can change to
                guided once script runs again. Therefore always change to position mode manually.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr

class controller():

    def __init__(self, spawn_position):

        self.error = 0.3
        self.flag = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_state = State()

        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def control(self):

        rate = rospy.Rate(20)

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        #offb_set_mode = SetModeRequest()
        #offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown() and not self.current_state.connected):

            rate.sleep()

        while(not rospy.is_shutdown()):

            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):

                rospy.sleep(1)

                #if(set_mode_client.call(offb_set_mode).mode_sent == True):

                    #rospy.loginfo("OFFBOARD enabled")

                #last_req = rospy.Time.now()

            #else:

            if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):

                if(arming_client.call(arm_cmd).success == True):

                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

            hover_mode.publisher()
            rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

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

    def state_callback(self, state_msg):

        self.current_state = state_msg


if __name__ == "__main__":

    try:

        rospy.init_node("drone_offb_node_py")
        hover_position = [5, 0, 5]
        spawn_position = [0, 0, 0]

        hover_mode = hover(hover_position, spawn_position)

        drone_controller = controller(spawn_position)
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
