#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from utils.motor_handler import MotorHandler
from utils.diff_drive_kinematics import DiffDriveKinematics

from geometry_msgs.msg import PointStamped
from acl_msgs.msg import ViconState
from controller.msg import Torques

class PIDController():
	"""
	Receives goal position and uses PID to publish torque commands for differential-drive
	robot to move towards it.
	"""
	def __init__(self):
		# PID parameters
		self.kp = rospy.get_param("kp")
		self.ki = rospy.get_param("ki")
		self.kd = rospy.get_param("kd")
		self.speed = rospy.get_param("speed")

		# Initialize needed errors and publisher
		now = rospy.Time.now()
		self.previous_time = now.secs + (float(now.nsecs)/10e8)
		self.previous_error = 0.0
		self.accumulated_error = 0.0
		self.torque_pub = rospy.Publisher("/torques", Torques, queue_size=1)

		# Keep track of position in world-frame
		self.position = None
		self.orientation = None # 2D unit vector of where the robot is facing in ground plane
		self.vicon_sub = rospy.Subscriber("/vicon", ViconState, self.update_pos)

		# Get goal position
		self.goal = None
		self.goal_sub = rospy.Subscriber("/clicked_point", PointStamped, self.update_goal)

		# Start control loop
		self.ddk = DiffDriveKinematics()
		END_THRESHOLD = 0.05 # meters
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			pos_error, heading_error = self.get_error()

			if heading_error is not None and np.linalg.norm(pos_error) > END_THRESHOLD:
				self.publish_control_input(heading_error)
			else:
				self.stop()

	def publish_control_input(self, heading_error):
		"""
		Takes heading error relative to robot and publishes necessary torque via PID.
		"""
		# PROPORTIONAL TERM
		p = self.kp * heading_error

		# INTEGRAL TERM
		self.accumulated_error += heading_error
		i = self.ki * self.accumulated_error

		# DERIVATIVE TERM
		now = rospy.Time.now()
		t = now.secs + (float(now.nsecs)/10e8)
		dt = t - self.previous_time
		d = self.kd * (heading_error - self.previous_error)/dt
		self.previous_time = t

		# Convert control input (rad/s) to torque (N*m)
		u = p + i + d
		lwm_omega, rwm_omega = self.ddk.robot_to_wheel(self.speed, u)
		lwm_torque = self.angular_velocity_to_torque(lwm_omega)
		rwm_torque = self.angular_velocity_to_torque(rwm_omega)

		# Publish torques
		trq = Torques()
		trq.lwm = lwm_torque
		trq.rwm = rwm_torque
		trq.lvm = 0.0
		trq.rvm = 0.0
		self.torque_pub.publish(trq)

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
		"""

		# TODO !!!!!!!!!!

		return omega

	def get_error(self):
		"""
		Return position error as 3D vector and heading error in horizontal plane.
		"""
		if self.position is None or self.goal is None:
			# Either robot position is unknown or goal is not set
			return None, None

		pos_error = self.goal - self.position
		u = pos_error[0:2]/np.linalg.norm(pos_error[0:2])
		heading_error = np.arccos(np.dot(self.orientation, u))

		return (pos_error, heading_error)

	def update_pos(self, msg):
		"""
		Update current known position and orientation using Vicon data.
			- msg: acl_msgs/ViconState
		"""
		p = msg.pose.position
		self.position = np.array([p.x, p.y, p.z])

		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
		self.orientation = np.array([np.cos(theta), np.sin(theta)])

	def update_goal(self, msg):
		"""
		Update current known goal.
			- msg: geometry_msgs/PointStamped
		"""
		g = msg.point
		self.goal = np.array([g.x, g.y, g.z])


if __name__ == '__main__':
	rospy.init_node("pid_control")
	pid = PIDController()
	rospy.on_shutdown(pid.stop)
	rospy.spin()
