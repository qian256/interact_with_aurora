#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage
import random
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import transformations



class NodeController():
	def __init__(self):
		rospy.init_node('console')
		self.imgPublisher = rospy.Publisher('/moverio/view/compressed', CompressedImage)
		self.poseSubscriber = rospy.Subscriber('/aruco/pose', Pose, self.newPose)
		self.seq = 0
		self.dist = 1.0
		self.ga = 0




	def newPose(self, poseMsg):
		self.publish()
		q = [poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z, poseMsg.orientation.w]
		rq = transformations.quaternion_matrix(q)
		al, be, ga = transformations.euler_from_matrix(rq, 'rxyz')
		self.ga = ga
		x = poseMsg.position.x
		y = poseMsg.position.y
		z = poseMsg.position.z
		self.dist = math.sqrt(x*x + y*y + z*z)
		print al, be, ga


	def publish(self):
		img = cv2.imread("/home/qian/dev/catkin_ws/src/smart_ar/script/aurora.png", cv2.CV_LOAD_IMAGE_COLOR)
		M = cv2.getRotationMatrix2D((img.shape[1]/2,img.shape[0]/2),self.ga*100, 1.0 / self.dist)
		img = cv2.warpAffine(img,M,(img.shape[1], img.shape[0]))
		bridge = CvBridge()
		msg = CompressedImage()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode(".jpg",img)[1]).tostring()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'image_compressed_publisher'
		msg.header.seq = self.seq
		self.seq += 1
		self.imgPublisher.publish(msg)
		print self.seq, 'image published'



if __name__ == '__main__':
	try:
		n = NodeController()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
