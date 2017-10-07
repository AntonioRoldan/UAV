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

x=0.0
y=0.0
z=0.0

def callback(position_msg):
    global x
    global y
    global z
    
    x=position_msg.pose.position.x
    y=position_msg.pose.position.y
    z=position_msg.pose.position.z

def main():
    rospy.init_node('check_position')

    listener = tf.TransformListener()
    rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, callback)
    success_pub = rospy.Publisher("/success", Int32, queue_size=1)

    rate = rospy.Rate(25)

    while not rospy.is_shutdown():

	try:
		(trans,rot) = listener.lookupTransform('/copter_home', '/copter_base_link', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	global x, y, z
	delta = 0.1
	if x<=trans[0]+delta and y<=trans[1]+delta and z<=trans[2]+delta:
		if x>=trans[0]-delta and y>=trans[1]-delta and z>=trans[2]-delta:
			print "Reach the position!"
			success_msg = Int32()
			success_msg.data = 1
			success_pub.publish(success_msg)		
	
		
        rate.sleep()

##########################
##########################

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
