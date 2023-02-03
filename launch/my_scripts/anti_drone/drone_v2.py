#!/usr/bin/env python3
import rospy
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import time
import tf.transformations
import math
import modern_robotics as mr
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class controller():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.error = 0.1

        self.flag = 0.0

        self.current_state = State()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        rospy.Subscriber("uav0/mavros/state", State, self.state_callback)

        rospy.Subscriber('/uav0/mavros/local_position/pose',
                            PoseStamped, self.position_callback)

    def control(self):

        rate = rospy.Rate(20)

        rospy.wait_for_service("/uav0/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("uav0/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/uav0/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("uav0/mavros/set_mode", SetMode)

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
                # trajectory_mode.control()
                circular_trajectory_mode.control()

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

        self.pose_publisher = rospy.Publisher('/uav0/mavros/local_position/pose',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.hover_x
        pose_msg.pose.position.y = self.hover_y
        pose_msg.pose.position.z = self.hover_z

        self.pose_publisher.publish(pose_msg)

class straight_trajectory():

    def __init__(self, trajectory, timegap, Xstart, Xend, samplingTime, Tf):

        self.trajectory = trajectory
        self.timegap = timegap
        self.time = 0.0
        self.Tf = Tf
        self.count = 0
        self.samplingTime = samplingTime

        self.pose_publisher = rospy.Publisher('/uav0/mavros/local_position/pose',
                                        Pose, queue_size = 10)


    def control(self):

        pose = self.trajectory[self.count]

        desired_x = pose[0][3]
        desired_y = pose[1][3]
        desired_z = pose[2][3]

        desired_position = np.mat(np.array([desired_x, desired_y, desired_z]))

        self.publisher(desired_x, desired_y, desired_z)

        self.count += 1

        if(self.count >= len(self.trajectory)):

            rospy.signal_shutdown("completed")

    def publisher(self, desired_x, desired_y, desired_z):

        # pose_msg = Pose()

        # pose_msg.position.x = desired_x
        # pose_msg.position.y = desired_y
        # pose_msg.position.z = desired_z

        # self.pose_publisher.publish(pose_msg)

        pose = PoseStamped()

        pose.header = std_msgs.msg.Header()

        pose.pose = Pose()

        pose.pose.position.x = desired_x
        pose.pose.position.y = desired_y
        pose.pose.position.z = desired_z

        self.pose_publisher.publish(pose)

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

        self.pose_publisher = rospy.Publisher('/uav0/mavros/local_position/pose',
                                        TwistStamped, queue_size = 10)

    def control(self):

        t = self.time/4

        self.desired_x = self.xc + self.radius*math.cos(2*math.pi*t/self.time_of_rev)
        self.desired_y = self.yc + self.radius*math.sin(2*math.pi*t/self.time_of_rev)
        self.desired_z = 3.0

        self.time += self.samplingTime

        self.publisher(self.desired_x, self.desired_y, self.desired_z)

    def publisher(self, desired_x, desired_y, desired_z):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = desired_x
        pose_msg.pose.position.y = desired_y
        pose_msg.pose.position.z = desired_z

        self.pose_publisher.publish(pose_msg)

if __name__ == '__main__':

    try:

        rospy.init_node('drone_node')

        hover_position = [0, 0, 5]

        traj_initial_pos = np.array(hover_position)
        traj_final_pos = np. array([20, 4, 6])

        Xstart = np.zeros((4, 4))
        Xend = np.zeros((4, 4))

        Xstart[:3, 3] = traj_initial_pos
        Xend[:3, 3] = traj_final_pos

        # straight line trajectory
        Tf = 12                                # time to reach from start to end position
        samplingTime = 1/100                    # sampling time in seconds
        N = int(Tf/samplingTime)                # number of samples
        method = 5                              # interpolation method
        trajectory = mr.CartesianTrajectory(
                Xstart, Xend, Tf, N, method)    # get the trajectory
        timegap = Tf / (N - 1.0)

        # circular trajectory

        centre = [0, 0]
        radius = 10.0
        time_of_rev = 8.0
        samplingTime = 1/100

        hover_mode = hover(hover_position)

        trajectory_mode = straight_trajectory(
            trajectory, timegap, Xstart, Xend, samplingTime, Tf)

        circular_trajectory_mode = circular_trajectory(centre, radius,
                                        time_of_rev, samplingTime)

        drone_controller = controller(hover_position)
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
