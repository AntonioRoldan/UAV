#!/usr/bin/env python
# -*- coding: latin-1 -*-
 
import rospy
import thread
import threading
from time import time
import mavros
import tf
import math
 
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int32


##########################
##########################

pubPosition = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) 

##########################
##########################

def main():
    rospy.init_node('follow_ar')
    mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(25)
    rospy.loginfo("Starting...")

    position_msg = PoseStamped()
    position_msg.header.frame_id = "copter_home"

    listener = tf.TransformListener()

    while not rospy.is_shutdown():
	
	try:
		(trans,rot) = listener.lookupTransform('/copter_home', '/ar_marker_13', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
	
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = trans[0]
        position_msg.pose.position.y = trans[1]
        position_msg.pose.position.z = trans[2] + 5

        pubPosition.publish(position_msg)
        rate.sleep()
    
    rospy.loginfo("Bye!")

##########################
##########################

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
