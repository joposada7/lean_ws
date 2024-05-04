#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from utils.diff_drive_kinematics import DiffDriveKinematics
from utils.controller import Controller

END_RADIUS = 0.05 # meters
RATE = rospy.Rate(60) # Hz

class LQRController(Controller):
	"""
	Receives goal position and uses LQR to publish torque commands for differential-drive
	robot to move towards it.
	"""
	def __init__(self):
		super().__init__()

		rospy.loginfo("Initializing LQR controller...")

		self.A = [0]
		self.B = [0]
		self.Q = [0] # State weighting matrix
		self.R = [0] # Control weighting matrix

		rospy.loginfo("LQR Controller initialized!")

	def publish_control_input(self):
		"""
		Calculate desired control input as torque (N*m) via LQR, and publishes it to self.torque_pub.
		"""
		raise NotImplementedError()



if __name__ == '__main__':
	rospy.init_node("lqr_control")
	lqr = LQRController()
	rospy.on_shutdown(lqr.stop)

	while not rospy.is_shutdown():
		lqr.control_loop(END_RADIUS)
		RATE.sleep()
