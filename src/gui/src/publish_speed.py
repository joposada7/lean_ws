#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from std_msgs.msg import Float64
from acl_msgs.msg import ViconState

class SpeedPublisher():
	"""
	Take Vicon state and publish speed of robot.
	Takes an average of speeds at a time for best logging.
	"""

	def __init__(self):		
		# Store position and time data for averaging
		self.MINIMUM_SPEEDS_NEEDED = 20
		self.times = np.array([])
		self.speeds = np.array([])
		self.thetas = np.array([])

		self.vicon_sub = rospy.Subscriber("/vicon", ViconState, self.publish_vicon_speed)
		self.speed_pub = rospy.Publisher("/speed", Float64, queue_size=1)

	def publish_vicon_speed(self, msg):
		"""
		Use velocity from ViconState Twist!
		"""
		t = msg.header.stamp
		time = t.secs + (float(t.nsecs)/10e9)

		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
		v = msg.twist.linear

		vel = np.array([v.x, v.y, v.z])
		direction = np.dot(vel, np.array([np.cos(theta), np.sin(theta), 0]))

		if direction > 0:
			vel = np.linalg.norm(vel)
		elif direction < 0:
			vel = -np.linalg.norm(vel)
		else:
			vel = 0.0

		if len(self.times) < self.MINIMUM_SPEEDS_NEEDED:
			self.times = np.append(self.times, time)
			self.speeds = np.append(self.speeds, vel)
			self.thetas = np.append(self.thetas, theta)
			return

		# Publish average
		self.speed_pub.publish(data=np.mean(self.speeds))

		# Cycle
		self.times = np.append(self.times[1:], time)
		self.speeds = np.append(self.speeds[1:], vel)
		self.thetas = np.append(self.thetas[1:], theta)

	def publish_speed(self, msg):
		t = msg.header.stamp
		time = t.secs + (float(t.nsecs)/10e9)

		p = msg.pose.position
		pos = np.array([p.x, p.y, p.z])
		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

		# Do nothing until we capture minimum needed
		if len(self.times) < self.MINIMUM_POSES_NEEDED:
			self.times = np.append(self.times, time)
			self.poses = np.append(self.poses, pos)
			self.thetas = np.append(self.thetas, theta)
			return
		self.poses = self.poses.reshape((self.MINIMUM_POSES_NEEDED,3))

		# Take average for speed
		speeds = []
		for i in range(self.MINIMUM_POSES_NEEDED-1):
			dt = self.times[i+1] - self.times[i]
			dp = self.poses[i+1] - self.poses[i]
			th = self.thetas[i]

			direction = np.array([np.cos(th), np.sin(th), 0])
			speed = np.dot(dp, direction)/dt
			speeds.append(speed)

		self.speed_pub.publish(data=np.mean(speeds))

		# Cycle
		self.times = np.append(self.times[1:], time)
		self.poses = np.append(self.poses[1:], pos)
		self.thetas = np.append(self.thetas[1:], theta)


if __name__ == "__main__":
	rospy.init_node("speed_publisher")
	sp = SpeedPublisher()
	rospy.spin()
	