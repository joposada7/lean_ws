#!/usr/bin/env python3
import rospy
import message_filters
import tf
import numpy as np

import os
import csv
from datetime import datetime

from std_msgs.msg import Float32, Float64
from acl_msgs.msg import ViconState

class IORecorder:
	"""
	For recording input/output of robot in Motion Capture room.
	Meant to be used for system identification, or just storing data for plots!
	"""

	def __init__(self):
		now = rospy.Time.now()
		self.t0 = now.secs + (float(rospy.Time.now().nsecs)/10e8) # Record start of node as time t=0

		# Store inputs
		self.times_i = []
		self.lwm_inputs = []
		self.rwm_inputs = []
		self.lvm_inputs = []
		self.rvm_inputs = []

		# Store outputs
		self.times_o = []
		self.velocity = []
		self.acceleration = []
		# self.rotation_rate = [] # TODO!!
		self.angular_velocity = []
		self.actuation_power = []
		self.computation_power = []

		# Inputs
		lwm_sub = message_filters.Subscriber("lwm_input", Float64)
		rwm_sub = message_filters.Subscriber("rwm_input", Float64)
		lvm_sub = message_filters.Subscriber("lvm_input", Float64)
		rvm_sub = message_filters.Subscriber("rvm_input", Float64)
		self.ts = message_filters.ApproximateTimeSynchronizer([lwm_sub, rwm_sub, lvm_sub, rvm_sub], queue_size=1, slop=0.05, allow_headerless=True)
		self.ts.registerCallback(self.record_input_only)

		# Outputs
		speed_sub = message_filters.Subscriber("/speed", Float64)
		ap_sub = message_filters.Subscriber("actuation_power", Float32)
		cp_sub = message_filters.Subscriber("computation_power", Float32)
		self.ts_i = message_filters.ApproximateTimeSynchronizer([speed_sub, ap_sub, cp_sub], queue_size=1, slop=0.05, allow_headerless=True)
		self.ts_i.registerCallback(self.record_output)

		rospy.on_shutdown(self.write_to_file) # Register shutdown hook
		rospy.loginfo("Ready to log!")

	def record_input(self, msg_lwm, msg_rwm, msg_lvm, msg_rvm):
		"""
		Record and store inputs from each of the motors.
			- msg_lwm: std_msgs/Float64
			- etc.
		"""
		# Timestamp
		now = rospy.Time.now()
		t = now.secs + (float(rospy.Time.now().nsecs)/10e8) - self.t0

		# Store
		self.times_i.append(t)
		self.lwm_inputs.append(msg_lwm.data)
		self.rwm_inputs.append(msg_rwm.data)
		self.lvm_inputs.append(msg_lvm.data)
		self.rvm_inputs.append(msg_rvm.data)

	def record_output(self, msg_v, msg_ap, msg_cp):
		"""
		Record position and calculate useful states.
			- msg_v:  std_msgs/Float64
			- msg_ap: std_msgs/Float64
			- msg_cp: std_msgs/Float64
		"""
		# Timestamp
		now = rospy.Time.now()
		t = now.secs + (float(rospy.Time.now().nsecs)/10e8) - self.t0

		"""
		# Current pose
		p = msg.pose.position
		pos = np.array([p.x, p.y, p.z])
		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
		"""

		vel = msg_v.data
		ap = msg_ap.data
		cp = msg_cp.data

		accel = 0
		omega = 0
		if len(self.times_o) > 0:
			dt = t - self.times_o[-1]

			# Acceleration
			prev_vel = self.velocity[-1]
			acc = (vel - prev_vel)/dt

			# Angular velocity
			RADIUS = 0.0127 # meters
			omega = np.linalg.norm(vel)/RADIUS

		# Store
		self.times_o.append(t)
		self.velocity.append(vel)
		self.acceleration.append(acc)
		self.angular_velocity.append(omega)
		self.actuation_power.append(ap)
		self.computation_power.append(cp)

	def write_to_file(self):
		"""
		Write all the data 2 large .csv files within the data directory corresponding to this run.
		"""
		# Sort data
		inputs = [self.times_i, self.lwm_inputs, self.rwm_inputs]
		inputs = self.pad_lists(inputs)
		data_i = np.column_stack(inputs)
		outputs = [self.times_o, self.velocity, self.acceleration, self.angular_velocity, self.actuation_power, self.computation_power]
		outputs = self.pad_lists(outputs)
		data_o = np.column_stack(outputs)

		# Save to folder in home directory
		d = os.path.dirname(os.path.expanduser('~'))
		datapath = os.path.join(d,os.getlogin(),'data')
		if not os.path.exists(datapath):
		    # If folder doesn't exist, make one
		    os.makedirs(datapath)

	    # Determine number for filename
		names = os.listdir(datapath)
		next_num = 1
		for name in names:
			if name.find("outputs")!=-1:
				# This is an outputs file
				s = name.split('-')
				num = int(s[-1][:-4])
				if num >= next_num:
					next_num = num+1
		filename = str(datetime.now().strftime("%m-%d"))+"-"+str(next_num)

		# Store inputs
		input_path = os.path.join(datapath,'inputs-'+filename+'.csv')
		with open(input_path, 'w+', newline='') as f:
			w = csv.writer(f)
			w.writerow(["t", "lwm", "rwm", "lvm", "rvm"]) # Header
			w.writerows(data_i)
		rospy.loginfo("Logged input data into " + input_path + "!")

		# Store ouputs
		output_path = os.path.join(datapath,'outputs-'+filename+'.csv')
		with open(output_path, 'w+', newline='') as f:
			w = csv.writer(f)
			w.writerow(["t", "v", "a", "omega", "acutation_power", "computation_power"]) # Header
			w.writerows(data_o)
		rospy.loginfo("Logged output data into " + output_path + "!")

		# Get relevant averages for torque-omega curve
		i0 = next(i for (i,dc) in enumerate(self.lwm_inputs) if dc > 0.0)
		i1 = next(i for (i,dc) in enumerate(self.lwm_inputs[i0:]) if dc == 0.0)
		t0 = self.times_i[i0]
		t1 = self.times_i[i1]
		rospy.loginfo(f"AVERAGE VELOCITY DURING TEST = {self.get_average(self.velocity, t0, t1)}")
		rospy.loginfo(f"AVERAGE ANGULAR VELOCITY DURING TEST = {self.get_average(self.angular_velocity, t0, t1)}")
		rospy.loginfo(f"AVERAGE ACUTATION POWER DURING TEST = {self.get_average(self.acutation_power, t0, t1)}")
		rospy.loginfo(f"AVERAGE COMPUTATION POWER DURING TEST = {self.get_average(self.computation_power, t0, t1)}")

	def get_average(self, lst, t0, t1):
		"""
		Get average value of a scalar list between times t0 and t1.
		"""
		# Determine if this is an input or output
		if len(lst) == len(self.times_i):
			times = np.array(self.times_i)
		elif len(lst) == len(self.times_o):
			times = np.array(self.times_o)
		else:
			rospy.logerr("Couldn't determine if this is an input or output!")
			return None

		# Get indices
		if t0 < times[0]:
			i0 = 0
		else:
			i0 = next(i for (i,t) in enumerate(times) if t >= t0)
		if t1 > times[-1]:
			i1 = len(times)-1
		else:
			i1 = next(i for (i,t) in enumerate(times) if t >= t1)

		return np.mean(lst[i0:i1])

	def pad_lists(self, lst):
		"""
		Turn arbitrary list of lists into numpy array of same lengths by padding ends with last elements.
		"""
		p = len(max(lst, key=len))
		if p == 0:
			return np.array(lst) # Empty lists

		return np.array([np.append(i,[i[-1]]*(p-len(i))) for i in lst])


if __name__ == '__main__':
	rospy.init_node("io_recorder")
	ior = IORecorder()
	rospy.spin()