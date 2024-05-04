#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from acl_msgs.msg import ViconState
from geometry_msgs.msg import PoseArray



# RELATIVE TO ROBOT STARTING POSITION
WAYPOINTS = np.array([
		[-3.20, -5.20, 0.652],
		[-2.70, -4.90, 0.652]
	])



if __name__ == "__main__":
	"""
	Transform waypoints in robot frame to world frame, then publish to robot.
	"""
	rospy.init_node("waypoint_publisher")
	rospy.loginfo("Gonna wait for message")
	# Get robot pose
	msg = rospy.wait_for_message("/vicon", ViconState)
	rospy.loginfo("Got message!")
	p = msg.pose.position
	o = msg.pose.orientation
	theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

	# Construct transformation matrix
	T = np.matrix([
		[np.cos(theta), -np.sin(theta), 0, p.x],
		[np.sin(theta), np.cos(theta),  0, p.y],
		[0,             0,              1, p.z],
		[0,             0,              0, 1]
	])
	rospy.loginfo("Constructed transformation matrix:")
	print(T)

	# Transform waypoints from robot frame --> world frame
	WORLD_WAYPOINTS = np.array([])
	for wp in WAYPOINTS:
		v = np.append(wp,1)
		v = v[:, np.newaxis]
		v_world = T*v
		WORLD_WAYPOINTS = np.append(WORLD_WAYPOINTS, v_world[:3,])
	WORLD_WAYPOINTS = WORLD_WAYPOINTS.reshape(len(WAYPOINTS), 3)
	
	# Publish as PoseArray
	wp_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)
	msg = PoseArray()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	for wp in WORLD_WAYPOINTS:
		msg.poses.append(wp)

	rospy.loginfo("World-frame waypoints given to robot:")
	print(msg.poses)
