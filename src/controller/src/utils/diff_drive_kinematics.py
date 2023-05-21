#!/usr/bin/env python3
import rospy
import numpy as np

class DiffDriveKinematics():
	"""
	Implements differential drive kinematic equations.
	"""
	def __init__(self):
		self.motor_width = rospy.get_param("motor_width") # meters
		self.wheel_radius = rospy.get_param("wheel_radius") # meters

	def wheel_to_robot(self, left_wheel_velocity, right_wheel_velocity):
		"""
		Takes wheel velocities in rad/s and returns the robot velocity
		and heading rate according to differential drive kinematics.
		"""
		robot_velocity = self.wheel_radius/2.0 * (left_wheel_velocity + right_wheel_velocity)
		heading_rate = self.wheel_radius/(2.0*self.motor_width) * (right_wheel_velocity - left_wheel_velocity)

		return [robot_velocity, heading_rate]

	def robot_to_wheel(self, velocity, heading_rate):
		"""
		Takes desired robot velocity and heading and returns the necessary
		wheel velocities in order to achieve them.
		"""
		total_wheel_velocity = 2.0/self.wheel_radius * velocity
		wheel_velocity_difference = (2.0*self.motor_width)/self.wheel_radius * heading_rate

		# Try to find set wheel velocities to match the desired heading rate
		left_wheel_velocity = total_wheel_velocity/2
		right_wheel_velocity = total_wheel_velocity/2
		while (np.sign(velocity)*left_wheel_velocity > 0) and (np.sign(velocity)*right_wheel_velocity > 0):
			[v, w] = self.wheel_to_robot(left_wheel_velocity, right_wheel_velocity)

			if abs(w - heading_rate) < np.deg2rad(0.05):
				# Within degree threshold
				return [left_wheel_velocity, right_wheel_velocity]

			# Try changing wheel velocities
			left_wheel_velocity -= 0.01*wheel_velocity_difference
			right_wheel_velocity += 0.01*wheel_velocity_difference

		# Nothing found without turning in place, just turn in place then
		return [-wheel_velocity_difference/2, wheel_velocity_difference/2]
