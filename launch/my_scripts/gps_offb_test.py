#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def position_callback(data):

    current_x = data.pose.position.x
    current_y = data.pose.position.y
    current_z = data.pose.position.z

    print(current_x, current_y, current_z)

def gps_callback(data):

    current_lat = data.latitude
    current_long = data.longitude
    current_alt = data.altitude

    #print(current_lat, current_long, current_alt)

if __name__ == "__main__":

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, gps_callback)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    global_pos_pub = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)

    rospy.Subsciber("/mavros/setpoint_position")

    rospy.Subscriber('mavros/local_position/pose', PoseStamped, position_callback)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):

        rate.sleep()

    vel_msg = TwistStamped()

    vel_msg.twist.linear.x = 0
    vel_msg.twist.linear.y = 0
    vel_msg.twist.linear.z = 0

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    gps_pose = GeoPoseStamped()

    pose.position.latitude =
    pose.position.longitude =
    pose.position.altitude =

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        #local_pos_pub.publish(pose)
        #local_vel_pub.publish(vel_msg)
        #gps_pose_pub.publish(gps_pose)

        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
                print("in")
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()


        #local_pos_pub.publish(pose)
        #local_vel_pub.publish(vel_msg)
	#gps_pose_pub.publish(gps_pose)

        rate.sleep()
