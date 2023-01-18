#! /usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

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

        rospy.Subscriber("mavros/state", State, self.state_callback)

        rospy.Subscriber('mavros/local_position/pose',
                            PoseStamped, self.position_callback)

    def control(self):

        rate = rospy.Rate(20)

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
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

            displacement = math.sqrt(pow((self.hover_x - self.current_x),2) +
                                        pow((self.hover_y - self.current_y),2)
                                        + pow((self.hover_z - self.current_z),2))

            if (displacement > self.error and self.flag == 0.0):

                hover_mode.publisher()
                rate.sleep()

            else:

                self.flag = 1.0
                circular_trajectory_mode.control()
                rate.sleep()

        # while not rospy.is_shutdown():

        #     displacement = math.sqrt(pow((self.hover_x - self.current_x),2) +
        #                                 pow((self.hover_y - self.current_y),2)
        #                                 + pow((self.hover_z - self.current_z),2))

        #     if (displacement > self.error):

        #         hover_mode.publisher()
        #         rate.sleep()

        #     else:

        #         while not rospy.is_shutdown():

        #             # circular_trajectory_mode.control()
        #             trajectory_mode.control()
        #             rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

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

class circular_trajectory():

    def __init__(self, centre, radius, time_of_rev, samplingTime):

        self.time = 0.0
        self.scale = 5.0
        self.xc = centre[0]
        self.yc = centre[1]
        self.radius = radius
        self.time_of_rev = time_of_rev
        self.samplingTime = samplingTime
        # self.desired_x = 0.0
        # self.desired_y = 0.0
        # self.desired_z = 0.0

        self.pose_publisher = rospy.Publisher('mavros/setpoint_position/local',
                                    PoseStamped, queue_size = 10)

    def control(self):

        t = 2*self.time

        self.desired_x = self.xc + self.radius*math.cos(2*math.pi*t/self.time_of_rev)
        self.desired_y = self.yc + self.radius*math.sin(2*math.pi*t/self.time_of_rev)
        self.desired_z = 2.0

        self.time += self.samplingTime

        self.publisher(self.desired_x, self.desired_y, self.desired_z)

    def publisher(self, desired_x, desired_y, desired_z):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = desired_x
        pose_msg.pose.position.y = desired_y
        pose_msg.pose.position.z = desired_z

        self.pose_publisher.publish(pose_msg)

if __name__ == "__main__":

    try:

        rospy.init_node("offb_node_py")

        hover_position = [0, 0, 2]
        hover_mode = hover(hover_position)

        centre = [0, 0]
        radius = 5.0
        time_of_rev = 8.0
        samplingTime = 1/100

        circular_trajectory_mode = circular_trajectory(centre, radius,
                                        time_of_rev, samplingTime)

        drone_controller = controller(hover_position)
        drone_controller.control()

    except rospy.ROSInterruptException:

        pass
