#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv

from nav_msgs.msg import OccupancyGrid

class MapSpawner():
	"""
	Imports a csv occupancy grid and publishes a single OccupancyGrid message to /map
	"""
	def __init__(self):
		self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

		self.width = 0
		self.height = 0
		self.occupancy = []

	def reset_map(self):
		"""
		Remove any currently imported map parameters.
		"""
		self.width = 0
		self.height = 0
		self.occupancy = []
		rospy.loginfo("Current map removed.")

	def import_map(self, filepath, max_value):
		"""
		Takes a .csv filepath and imports map to self parameters.

		- max_value: float, maximum occupancy grid value (i.e. occupied value)
		"""
		if not os.path.isfile(filepath):
			rospy.logerr("File '" + str(filepath) + "' does not exist.")
			return False
		if filepath[-4:] != ".csv":
			rospy.logerr("Expected .csv for occupancy grid!")
			return False

		if len(self.occupancy) > 0:
			rospy.loginfo("Overwriting previously imported map.")
			self.reset_map()

		with open(filepath, 'r') as f:
			cf = csv.reader(f)
			for row in cf:
				self.width += 1
				for occ in row:
					scaled = self.scale(float(occ), max_value)
					if scaled < 0 or scaled > 100:
						rospy.logerr("Occupancy grid values out of bounds!")
						self.reset_map()
						return False
					self.occupancy.append(int(scaled))
			self.height = int(len(self.occupancy)/self.width)

		if self.width*self.height != len(self.occupancy):
			rospy.logerr("Imported map is not a square!")
			self.reset_map()
			return False
		self.to_column_major()
		rospy.loginfo("Map imported!")

	def scale(self, value, max_value):
		"""
		Scale value from 0-max_value to 0-100.
		"""
		return value * 100.0/max_value

	def to_column_major(self):
		"""
		Convert occupancy list from row major to column major ordering.
		"""
		self.occupancy = list(np.array(self.occupancy).reshape(self.width, self.height).flatten('F'))

	def publish_map(self):
		"""
		Publish OccupancyGrid msg to /map
		"""
		msg = OccupancyGrid()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "world"

		msg.info.map_load_time = msg.header.stamp
		msg.info.resolution = 1.0/rospy.get_param("pixels_per_meter")
		msg.info.width = self.width
		msg.info.height = self.height
		# msg.info.width = int(self.width*rospy.get_param("pixels_per_meter"))
		# msg.info.height = int(self.height*rospy.get_param("pixels_per_meter"))

		msg.info.origin.position.x = -rospy.get_param("start_x")
		msg.info.origin.position.y = -rospy.get_param("start_y")
		msg.info.origin.position.z = -rospy.get_param("start_z")

		# msg.info.origin.orientation.x = 
		# msg.info.origin.orientation.y = 
		# msg.info.origin.orientation.z = 
		# msg.info.origin.orientation.w = 

		msg.data = self.occupancy
		self.map_pub.publish(msg)
		rospy.loginfo("Map published!")



if __name__ == '__main__':
	rospy.init_node("map_spawner")
	ms = MapSpawner()

	MAP_PATH = rospy.get_param("map_path")
	ms.import_map(MAP_PATH, max_value=255)

	rospy.sleep(5.0) # TODO: Make this just wait for rviz to load
	ms.publish_map()
	# rospy.spin()