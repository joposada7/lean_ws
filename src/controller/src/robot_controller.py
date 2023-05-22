#!/usr/bin/env python3
import rospy
from utils.motor_handler import MotorHandler

from controller.msg import Torques

class RobotController():
	"""
	Subscribes to torque messages and moves motors as specified.
	"""
	def __init__(self):
		# Robot limits
		self.MAX_SPEED = rospy.get_param("max_speed") # m/s
		self.MAX_ROTATION_RATE = rospy.get_param("max_rotation_rate") # rad/s
		self.MAX_TORQUE = rospy.get_param("max_torque") # N*m

		# Initialize motors and begin listening for torque commands
		self.motors = MotorHandler()
		self.torque_sub = rospy.Subscriber("/torques", Torques, self.move)

	def move(self, msg):
		"""
		Spin motors as specified by torques.
			- msg: controller/Torques
		"""
		lwm_dc = self.torque_to_duty_cycle(msg.lwm)
		rwm_dc = self.torque_to_duty_cycle(msg.rwm)
		lvm_dc = self.torque_to_duty_cycle(msg.lvm)
		rvm_dc = self.torque_to_duty_cycle(msg.rvm)

		self.motors.LWM.change_duty_cycle(lwm_dc)
		self.motors.RWM.change_duty_cycle(rwm_dc)
		self.motors.LVM.change_duty_cycle(lvm_dc)
		self.motors.RVM.change_duty_cycle(rvm_dc)

	def torque_to_duty_cycle(self, torque):
		"""
		Convert a given torque (N*m) to duty cycle between -100% to 100%.
		If torque is above motor limits, returns 100% and warns.
		"""

		# TODO!!!!!!!

		rospy.loginfo(f"Desired torque={torque} N*m")
		if torque > 100:
			return 100
		elif torque < -100:
			return -100
		else:
			return torque


if __name__ == "__main__":
	rospy.init_node("robot_controller")
	rc = RobotController()
	rospy.on_shutdown(rc.motors.cleanup_motors) # Cleanup GPIO on node shutdown
	rospy.spin()
