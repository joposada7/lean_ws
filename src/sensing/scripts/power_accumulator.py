#!/usr/bin/env python3
import rospy
import message_filters

from std_msgs.msg import Float32

class PowerAccumulator:
	"""
	Object used to track total energy usage over a particular time period.
	E.g.:

	pa = PowerAccumulator()
	pa.start()
	# Do something
	pa.stop()
	total_energy = pa.get_actuation_energy() + pa.get_computation_energy()
	"""

	def __init__(self):
		self.currentActuationEnergy = 0.0
		self.currentComputationEnergy = 0.0
		self.startTime = None
		self.stopTime = None
		self.previousTime = None

	def start(self):
		"""
		Start logging power.
		"""
		if self.stopTime is not None:
			raise ValueError("This accumulator has stopped logging.")

		ap_sub = message_filters.Subscriber("actuation_power", Float32)
		cp_sub = message_filters.Subscriber("computation_power", Float32)
		self.power_sub = message_filters.ApproximateTimeSynchronizer([ap_sub, cp_sub], queue_size=1, slop=0.05, allow_headerless=True)
		self.power_sub.registerCallback(self.log_power)

		self.startTime = self.get_time()
		self.previous_time = self.startTime

	def stop(self):
		"""
		Stop logging power. No more power can be logged on this object.
		"""
		[sub_obj.sub.unregister() for sub_obj in self.power_subs]
		self.stopTime = self.get_time()

	def log_power(self, msg_ap, msg_cp):
		"""
		Receive power data, add to current energy accumulation.
		"""
		t = self.get_time()
		dt = t - self.previousTime

		# Get differentials in comp/act energy
		dae = msg_ap.data/dt
		dce = msg_cp.data/dt

		self.currentActuationEnergy += dae
		self.currentComputationEnergy += dce

	def get_actuation_energy(self):
		return self.currentActuationEnergy

	def get_computation_energy(self):
		return self.currentComputationEnergy

	def get_average_actuation_power(self):
		return self.currentActuationEnergy/self.get_running_time()

	def get_average_computation_power(self):
		return self.currentComputationEnergy/self.get_running_time()

	def get_running_time(self):
		"""
		Get time the accumulator has been running.
		"""
		if self.stopTime is None:
			return self.get_time() - self.startTime
		return self.stopTime - self.startTime

	def get_time(self):
		"""
		Convert current ROS time to float.
		"""
		now = rospy.Time.now()
		t = now.secs + (float(now.nsecs)/10e8)
		return t
