#!/usr/bin/env python
#!coding=utf-8
 
#right code !
#write by leo at 2018.04.26
#function: 
#display the frame from another node.
import base64
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import zmq

def callback(data):
	
	state[0] = data.pose.position.x
	state[1] = data.pose.position.y
	state[2] = data.pose.position.z
	state[3] = data.pose.orientation.w
	state[4] = data.pose.orientation.x
	state[5] = data.pose.orientation.y
	state[6] = data.pose.orientation.z
	print(state)
	
 
def receive():
	rospy.init_node('send_display', anonymous=True)
	# make a video_object and init the video object
	global state = np.array([0.0,0,0,0,0,0,0])

	rospy.Subscriber('vrpn_client_node/armdrone/pose', PoseStamped, callback)
	rospy.spin()
 
if __name__ == '__main__':
	receive()
