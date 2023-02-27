#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr

class controller():

    def __init__(self, waypoints):

        self.error = 0.3

        self.flag = 0.0

        self.i = 0
        self.length = len(waypoints)

        self.hover_x = 0.0
        self.hover_y = 0.0
        self.hover_z = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_state = State()

        rospy.Subscriber("mavros/state", State, self.state_callback)

        rospy.Subscriber('mavros/local_position/pose',
                            PoseStamped, self.position_callback)

        self.pose_publisher = rospy.Publisher('mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

    def control(self):

        rate = rospy.Rate(20)

        rospy.wait_for_service("mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

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

            while(self.length > 0 and not rospy.is_shutdown() and arming_client.call(arm_cmd).success == True):

                # print("in")

                # pose = PoseStamped()

                # pose.pose.position.x = 0
                # pose.pose.position.y = 0
                # pose.pose.position.z = 4

                # self.pose_publisher.publish(pose)
                self.waypoint_publisher(waypoints)

    def waypoint_publisher(self, waypoints):

        x = waypoints[self.i, 0]
        y = waypoints[self.i, 1]
        z = waypoints[self.i, 2]

        self.publisher(x, y, z)

        displacement = math.sqrt(pow((x - self.current_x),2) +
                            pow((y - self.current_y),2)
                            + pow((z - self.current_z),2))


        if(displacement < self.error and self.i == len(waypoints) - 1):

            print("land")
            rospy.wait_for_service("mavros/cmd/landing")
            land_client = rospy.ServiceProxy("mavros/cmd/landing", CommandBool)

            land_cmd = CommandBoolRequest()
            land_cmd.value = True

            rospy.signal_shutdown("script completed")

        elif(displacement > self.error):

            rospy.sleep(1)

        else:

            self.i += 1

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

    def publisher(self, hover_x, hover_y, hover_z):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = hover_x
        pose_msg.pose.position.y = hover_y
        pose_msg.pose.position.z = hover_z

        print("Publishing", self.i, "th waypoint")

        self.pose_publisher.publish(pose_msg)

class hover():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.pose_publisher = rospy.Publisher('mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.hover_x
        pose_msg.pose.position.y = self.hover_y
        pose_msg.pose.position.z = self.hover_z

        self.pose_publisher.publish(pose_msg)

if __name__ == "__main__":

    try:

        rospy.init_node("offb_node_py")

        waypoints = np.mat(np.array([[0,0,5], [0,5,5], [0,0,5]]))

        drone_controller = controller(waypoints)
        drone_controller.control()

    except rospy.ROSInterruptException:

        pass
