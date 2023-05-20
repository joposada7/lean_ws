#!/usr/bin/env python
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from acl_msgs.msg import ViconState

class TFPublisher():
	"""
	Take Vicon state and publish pose as transform for /world frame.
	"""

	def __init__(self):
		self.vicon_sub = rospy.Subscriber("/vicon", ViconState, self.publish_tf)
		self.br = tf2_ros.TransformBroadcaster()

	def publish_tf(self, msg):
		t = TransformStamped()
		t.header.stamp = msg.header.stamp
		t.header.frame_id = "/world"
		# t.child_frame_id = msg.header.frame_id
		t.child_frame_id = "/lean"
		t.transform.translation = msg.pose.position
		t.transform.rotation = msg.pose.orientation
		rospy.loginfo(t)
		self.br.sendTransform(t)

if __name__ == "__main__":
	rospy.init_node("vicon_tf_publisher")
	tfp = TFPublisher()
	rospy.spin()
	