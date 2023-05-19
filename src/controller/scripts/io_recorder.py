#!/usr/bin/env python3
import rospy
import message_filters
import tf
import numpy as np

import os
import csv
from datetime import datetime

from std_msgs.msg import Float64
from acl_msgs.msg import ViconState

class IORecorder:
	"""
	For recording input/output of robot in Motion Capture room.
	Meant to be used for system identification, or just storing data for plots!
	"""

	def __init__(self):
		now = rospy.Time.now()
		self.t0 = now.secs + (float(rospy.Time.now().nsecs)/10e9) # Record start of node as time t=0

		# Store data
		self.times_i = []
		self.lwm_inputs = []
		self.rwm_inputs = []
		self.lvm_inputs = []
		self.rvm_inputs = []

		self.times_o = []
		self.positions = []
		self.velocities = []
		self.accelerations = []
		self.orientations = []
		self.rotation_rates = []
		self.angular_velocity = []

		# Outputs
		pose_sub = rospy.Subscriber("vicon", ViconState, self.record_pose_only)

		# Inputs
		lwm_sub = message_filters.Subscriber("lwm_input", Float64)
		rwm_sub = message_filters.Subscriber("rwm_input", Float64)
		lvm_sub = message_filters.Subscriber("lvm_input", Float64)
		rvm_sub = message_filters.Subscriber("rvm_input", Float64)
		THRESHOLD = rospy.get_param("msg_arrival_threshold")
		self.ts = message_filters.ApproximateTimeSynchronizer([lwm_sub, rwm_sub, lvm_sub, rvm_sub], queue_size=1, slop=THRESHOLD, allow_headerless=True)
		self.ts.registerCallback(self.record_input_only)

		rospy.on_shutdown(self.write_to_file) # Register shutdown hook
		rospy.loginfo("Ready to log!")

	def record_input_only(self, msg_lwm, msg_rwm, msg_lvm, msg_rvm):
		"""
		Record and store inputs from lwm and rwm.
			- msg_lwm: std_msgs/Float64
			- msg_rwm: std_msgs/Float64
		"""
		# Timestamp
		now = rospy.Time.now()
		t = now.secs + (float(rospy.Time.now().nsecs)/10e9) - self.t0

		# Store
		self.times_i.append(t)
		self.lwm_inputs.append(msg_lwm.data)
		self.rwm_inputs.append(msg_rwm.data)
		self.lvm_inputs.append(msg_lvm.data)
		self.rvm_inputs.append(msg_rvm.data)

	def record_pose_only(self, msg):
		"""
		Record pose and calculate useful states.
			- msg: acl_msgs/ViconState
		"""
		# Timestamp
		now = rospy.Time.now()
		t = now.secs + (float(rospy.Time.now().nsecs)/10e9) - self.t0

		# Current pose
		p = msg.pose.position
		pos = np.array([p.x, p.y, p.z])
		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

		# Find derivatives
		if len(self.times_o) > 0:
			dt = t - self.times_o[-1]

			prev_pos = self.positions[-1]
			vel = (pos - prev_pos)/dt
			prev_vel = self.velocities[-1]
			acc = (vel - prev_vel)/dt
			prev_theta = self.orientations[-1]
			rate = (theta - prev_theta)/dt

			RADIUS = 0.0127 # meters
			omega = np.linalg.norm(vel)/RADIUS
		else:
			vel = [0,0,0]
			acc = [0,0,0]
			rate = 0
			omega = 0

		# Store
		self.times_o.append(t)
		self.positions.append(pos)
		self.velocities.append(vel)
		self.accelerations.append(acc)
		self.orientations.append(theta)
		self.rotation_rates.append(rate)
		self.angular_velocity.append(omega)

	def write_to_file(self):
		"""
		Write all the data 2 large .csv files within the data directory corresponding to this run.
		"""
		pos = np.array(self.positions)
		vel = np.array(self.velocities)
		acc = np.array(self.accelerations)

		data_i = np.c_[self.times_i, self.lwm_inputs, self.rwm_inputs]
		data_o = np.c_[self.times_o, pos[:,0], pos[:,1], pos[:,2], vel[:,0], vel[:,1], vel[:,2], acc[:,0], acc[:,1], acc[:,2], self.orientations, self.rotation_rates, self.angular_velocity]

		d = os.path.dirname(__file__)
		datapath = os.path.join(d,'..','data')
		numfiles = len(os.listdir(datapath)) # Should be in pairs
		num = int(numfiles/2)+1
		filename = str(datetime.now().strftime("%m-%d"))+"-"+str(num)

		# Store inputs
		input_path = os.path.join(d,'..','data','inputs-'+filename+'.csv')
		with open(input_path, 'w+', newline='') as f:
			w = csv.writer(f)
			w.writerow(["t", "lwm", "rwm"]) # Header
			w.writerows(data_i)
		rospy.loginfo("Logged input data into " + input_path + "!")

		# Store ouputs
		output_path = os.path.join(d,'..','data','outputs-'+filename+'.csv')
		with open(output_path, 'w+', newline='') as f:
			w = csv.writer(f)
			w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "theta", "theta_dot", "omega"]) # Header
			w.writerows(data_o)
		rospy.loginfo("Logged output data into " + output_path + "!")


if __name__ == '__main__':
	rospy.init_node("io_recorder")
	ior = IORecorder()
	rospy.spin()