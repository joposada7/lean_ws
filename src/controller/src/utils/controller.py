#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from abc import ABC, abstractmethod

from utils.viz_tools import VisualizationTools

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Joy
from acl_msgs.msg import ViconState
from controller.msg import Torques

class Controller(ABC):
	"""
	Abstract class that defines generic controller loop.
	Initializes:
		- Visualization tools
		- Position/Orientation of robot in world-frame
		- Goal position
		- Joystick
		- Torque publisher
	Need to implement before initalizing an object:
		- publish_control_input(self)
	"""
	def __init__(self):
		# Initialize visualization tools
		self.vt = VisualizationTools()

		# Keep track of position in world-frame
		self.position = None
		self.orientation = None
		vicon_sub = rospy.Subscriber("/vicon", ViconState, self.update_pos)

		# Get goal position
		self.goal = None
		goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal)

		# Listen to joystick
		self.kill_switch_pressed = False
		joy_sub = rospy.Subscriber("/joy", Joy, self.check_for_kill_switch, queue_size=1)

		# Setup torque input publisher
		self.torque_pub = rospy.Publisher("/torques", Torques, queue_size=1)

	def check_for_kill_switch(self, msg):
		"""
		Check to see if A button is pressed.
			- msg: sensor_msgs/Joy
		"""
		if msg.buttons[0]:
			if not self.kill_switch_pressed:
				rospy.loginfo("Detected kill switch pressed.")
			self.kill_switch_pressed = True
		else:
			self.kill_switch_pressed = False

	def stop(self):
		"""
		Publish no torque command.
		"""
		trq = Torques()
		trq.lwm = 0.0
		trq.rwm = 0.0
		trq.lvm = 0.0
		trq.rvm = 0.0
		self.torque_pub.publish(trq)

	def angular_velocity_to_torque(self, omega):
		"""
		Convert between angular velocity and torque using experimentally derived relation.
			- omega: angular velocity (rad/s)
		"""
		# TODO!!!
		raise NotImplementedError()

	def get_error(self):
		"""
		Return 2-tuple of position error as 3D vector and heading error in horizontal plane.
		"""
		if self.position is None or self.goal is None:
			# Either robot position is unknown or goal is not set
			return None, None

		pos_error = self.goal - self.position
		heading_error = self.get_signed_angle(self.orientation, pos_error)
		return (pos_error, heading_error)

	def get_signed_angle(self, vec1, vec2):
		"""
		Get the signed angle between two signed 3D vectors. Assumes normal is Z+ [0,0,1].
		"""
		NORMAL = [0,0,1]
		return np.arctan2(np.dot(np.cross(vec1, vec2), NORMAL), np.dot(vec1, vec2))

	def update_pos(self, msg):
		"""
		Update current known position and orientation using Vicon data.
			- msg: acl_msgs/ViconState
		"""
		p = msg.pose.position
		self.position = np.array([p.x, p.y, p.z])

		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
		self.orientation = np.array([np.cos(theta), np.sin(theta), 0])

		self.vt.draw_arrow("position", self.position, [o.x, o.y, o.z, o.w])

	def update_goal(self, msg):
		"""
		Update current known goal.
			- msg: geometry_msgs/PoseStamped
		"""
		g = msg.pose.position
		self.goal = np.array([g.x, g.y, g.z])
		self.vt.draw_point("goal", self.goal, rgb=[0.0,0.9,0.1])

	def control_loop(self, END_RADIUS):
		"""
		Check for goal and publish a control input if robot is not within END_RADIUS (m) of the goal.
		"""
		if self.goal is None:
			# No goal yet
			self.stop()
		elif np.linalg.norm(pos_error) > END_RADIUS:
			# Haven't reached goal yet
			self.publish_control_input()
		else:
			# Reached goal
			lqr.goal = None
			lqr.stop()



	#------------------- ABSTRACT METHODS ------------------------

	@abstractmethod
	def publish_control_input(self):
		"""
		Calculates a control input as a torque (N*m), then publishes it to self.torque_pub.
		"""
		pass
