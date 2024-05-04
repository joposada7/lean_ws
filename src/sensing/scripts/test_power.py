#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class PowerTester():
	"""
	Quick script to test that power sensing capabilities are working on the Pi.
	"""
	def __init__(self):
		"""
		Initialize subscribers to power and voltage topics.
		"""
		ap_sub = rospy.Subscriber("actuation_power", Float32, self.sense_callback(type="actuation power", unit="W"))
		cp_sub = rospy.Subscriber("computation_power", Float32, self.sense_callback(type="computation power", unit="W"))
		av_sub = rospy.Subscriber("actuation_voltage", Float32, self.sense_callback(type="actuation voltage", unit="V"))
		cv_sub = rospy.Subscriber("computation_voltage", Float32, self.sense_callback(type="computation voltage", unit="V"))

	def sense_callback(self, type="unknown power type", unit=""):
		"""
		Return a callback function that logs a sensing message (Float32).
		"""
		def log(msg):
			rospy.loginfo(f"Received {type} = {msg.data} {unit}")
		return log

if __name__ == '__main__':
	rospy.init_node("power_tester")
	pt = PowerTester()
	rospy.spin()