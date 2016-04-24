#!/usr/bin/env python

import roslib
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


class NodeCalibUpdator():
	def __init__(self):
		self.seq = 0;
		rospy.init_node('calibration_updator')
		self.publisher = rospy.Publisher('/updated/camera_info', CameraInfo)
		self.subscriber = rospy.Subscriber('/camera/camera_info', CameraInfo, self.newInfo)

		self.createMsg()


	def createMsg(self):
		msg = CameraInfo()
		msg.height = 480
		msg.width = 640
		msg.distortion_model = "plumb_bob"
		msg.D = [0.08199114285264993, -0.04549390835713936, -0.00040960290587863145, 0.0009833748346549968, 0.0]
		msg.K = [723.5609128875188, 0.0, 315.9845354772031, 0.0, 732.2615109685506, 242.26660681756633,\
				 0.0, 0.0, 1.0]
		msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		msg.P = [737.1736450195312, 0.0, 316.0324931112791, 0.0, 0.0, 746.1573486328125, 241.59042622688867,\
				 0.0, 0.0, 0.0, 1.0, 0.0]
		self.msg = msg



	def newInfo(self, infomsg):
		self.seq = self.seq + 1
		self.msg.header = infomsg.header
		self.publisher.publish(self.msg)



if __name__ == '__main__':
	try:
		n = NodeCalibUpdator()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

