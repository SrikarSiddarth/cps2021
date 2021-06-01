#!/usr/bin/env python
import rospy 
from numpy import array
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class detectUAV():
	def __init__(self):
		rospy.init_node('detectUAV')
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/exocam/cam_1/image_raw', Image, callback=self.image_cb)
		self.img = None
		rospy.spin()

	def image_cb(self, msg):
		try:
			rec = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			rec = cv.resize(rec,(720,540),cv.INTER_AREA)
			self.img = rec[100:,:]
			hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
			low = array([0,0,0])
			up = array([0,0,40])
			mask = cv.inRange(hsv, low, up)
			_, contour,_ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
			print(len(contour))
			cv.imshow('d',mask)
			cv.waitKey(33)
		except CvBridgeError as e:
			print(e)



if __name__ == '__main__':
	d = detectUAV()