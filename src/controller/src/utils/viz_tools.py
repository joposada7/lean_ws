#!/usr/bin/env python3

import rospy
import numpy as np

from visualization_msgs.msg import Marker

class VisualizationTools():
	"""
	Implements visualization tools.
	"""

	def __init__(self):
		self.viz_pub = rospy.Publisher("/viz", Marker, queue_size=1)
		self.WORLD_FRAME = "world"

	def draw_point(self, ns, pos, rgb=[0.9,0.0,0.1]):
		"""
		Draw marker at given point.
			- ns:  Namespace of viz
			- pos: [x, y, z] Position (meters)
		"""
		m = Marker()
		m.header.stamp = rospy.Time.now()
		m.header.frame_id = self.WORLD_FRAME
		m.ns = ns
		m.id = 0
		m.type = 2 # SPHERE
		m.action = 0 # ADD

		m.pose.position.x = pos[0]
		m.pose.position.y = pos[1]
		m.pose.position.z = pos[2]
		m.pose.orientation.x = 0.0
		m.pose.orientation.y = 0.0
		m.pose.orientation.z = 0.0
		m.pose.orientation.w = 1.0
		
		m.scale.x = 0.15
		m.scale.y = 0.15
		m.scale.z = 0.15
		m.color.a = 1.0
		m.color.r = rgb[0]
		m.color.g = rgb[1]
		m.color.b = rgb[2]

		self.viz_pub.publish(m)

	def draw_arrow(self, ns, pos, ori, rgb=[0.9,0.0,0.1]):
		"""
		Draw arrow at given pose.
			- ns:  Namespace of viz
			- pos: [x, y, z] Position (meters)
			- ori: [x, y, z, w] Quaternion
		"""
		m = Marker()
		m.header.stamp = rospy.Time.now()
		m.header.frame_id = self.WORLD_FRAME
		m.ns = ns
		m.id = 0
		m.type = 0 # ARROW
		# m.type = 2 # SPHERE
		m.action = 0 # ADD

		m.pose.position.x = pos[0]
		m.pose.position.y = pos[1]
		m.pose.position.z = pos[2]
		m.pose.orientation.x = ori[0]
		m.pose.orientation.y = ori[1]
		m.pose.orientation.z = ori[2]
		m.pose.orientation.w = ori[3]
		
		m.scale.x = 0.3
		m.scale.y = 0.04
		m.scale.z = 0.04
		m.color.a = 1.0
		m.color.r = rgb[0]
		m.color.g = rgb[1]
		m.color.b = rgb[2]

		self.viz_pub.publish(m)