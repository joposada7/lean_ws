#!/usr/bin/env python3
import rospy
import message_filters
import tf

import csv
from datetime import date

from std_msgs.msg import Float64
from acl_msgs.msg import ViconState

class IORecorder:
	"""
	For recording input/output of robot in Motion Capture room.
	Meant to be used for system identification, or just storing data for plots!
	"""

	def __init__(self):
		# Both
		"""
		INPUT_TOPIC = rospy.get_param("input_topic")
		VICON_TOPIC = rospy.get_param("vicon_topic")

		voltage_sub = message_filters.Subscriber(INPUT_TOPIC, Float64)
		pose_sub = message_filters.Subscriber(VICON_TOPIC, ViconState)

		# Wait for messages to line up within threshold
		THRESHOLD = rospy.get_param("msg_arrival_threshold")
		self.ts = message_filters.TimeSynchronizer([voltage_sub, pose_sub], slop=THRESHOLD)
		self.ts.registerCallback(self.record)
		"""

		# One
		VICON_TOPIC = rospy.get_param("vicon_topic")
		pose_sub = rospy.Subscriber(VICON_TOPIC, ViconState, self.record_pose_only)

		# Store data
		self.times = []
		self.inputs = []
		self.positions = []
		self.velocities = []
		self.accelerations = []
		self.orientations = []
		self.rotation_rates = []
		rospy.on_shutdown(self.write_to_file) # Register shutdown hook

	def record(self, i_msg, o_msg):
		"""
		Record input/output and calculate useful states.
			- i_msg: std_msgs/Float64
			- o_msg: acl_msgs/ViconState
		"""
		# Timestamp
		t = rospy.Time.now().secs

		# Input
		i = 0.0
		rospy.logwarn("No input found!!")

		# Current pose
		p = o_msg.pose.position
		pos = np.array([p.x, p.y, p.z])
		o = o_msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

		# Find derivatives
		if len(self.times) > 0:
			dt = t - self.times[-1]

			prev_pos = self.positions[-1]
			vel = (pos - prev_pos)/dt
			prev_vel = self.velocities[-1]
			acc = (vel - prev_vel)/dt
			prev_theta = self.orientations[-1]
			rate = (theta - prev_theta)/dt
		else:
			vel = [0,0,0]
			acc = [0,0,0]
			rate = 0

		# Store
		self.times.append(t)
		self.positions.append(pos)
		self.velocities.append(vel)
		self.accelerations.append(acc)
		self.orientations.append(theta)
		self.rotation_rates.append(rate)

	def record_pose_only(self, msg):
		"""
		Record pose and calculate useful states.
			- msg: acl_msgs/ViconState
		"""
		# Timestamp
		t = rospy.Time.now().secs

		# Current pose
		p = msg.pose.position
		pos = np.array([p.x, p.y, p.z])
		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

		# Find derivatives
		if len(self.times) > 0:
			prev_pos = self.positions[-1]
			vel = (pos - prev_pos)/t
			prev_vel = self.velocities[-1]
			acc = (vel - prev_vel)/t
			prev_theta = self.orientations[-1]
			rate = (theta - prev_theta)/t
		else:
			vel = [0,0,0]
			acc = [0,0,0]
			rate = 0

		# Store
		self.times.append(t)
		self.positions.append(pos)
		self.velocities.append(vel)
		self.accelerations.append(acc)
		self.orientations.append(theta)
		self.rotation_rates.append(rate)

	def write_to_file(self):
		"""
		Write all the data to a large .csv file within the data directory corresponding to this run.
		"""
		data = np.c_[self.times, self.positions, self.velocities, self.accelerations, self.orientations, self.rotation_rates]
		filename = str(date.today()) + "-io-recording"

		with open("./data/"+filename+".csv", 'w', newline='') as f:
			w = csv.writer(f)
			w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "theta", "theta_dot"]) # Header
			w.writerows(data)


if __name__ == '__main__':
	rospy.init_node("io_recorder")
	ior = IORecorder()
	rospy.spin()