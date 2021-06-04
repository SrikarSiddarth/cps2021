#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode, CommandBool, ParamSet
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class OffBoardControl():
	def __init__(self):
		rospy.init_node('OffboardControlPhase1', anonymous=True)
		self.curr_pose = PoseStamped()
		self.des_pose = PoseStamped()
		self.mode = "BELLY-FLOP"
		self.arm = True

		self.armRover = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
		self.roverModeService = rospy.ServiceProxy('/uav0/mavros/set_mode',SetMode)

		self.roverStateSub = rospy.Subscriber('/uav0/mavros/state', State, callback=self.rover_state_callback)

		self.controller()


	def rover_state_callback(self,msg):
		if msg.mode!='OFFBOARD':
			print('switching to offboard')
			self.roverModeService(custom_mode='OFFBOARD')

		if self.arm != msg.armed:
			print('arming/g/disarming the rover')
			self.armRover(self.arm)

	def controller(self):
		# rate = rospy.Rate(15)
		# while not rospy.is_shutdown():

		# 	rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	OffBoardControl()