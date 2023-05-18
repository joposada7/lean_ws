#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Log

NODE_NAME = "roslogger"

class ROSLogger():
	"""
	Take messages from /rosout and rebroadcast them.
	Meant for simpler logging from remote nodes.
	"""
	def __init__(self):
		self.rosout_sub = rospy.Subscriber("rosout", Log, self.rebroadcast, queue_size=10)

	def rebroadcast(self, msg):
		"""
		Rebroadcast message based on level. logwarns unknown types.
		"""
		if msg.name == "/"+NODE_NAME:
			return # Skip logs from this node!

		if msg.level == msg.DEBUG:
			rospy.logdebug(msg.msg)
		elif msg.level == msg.INFO:
			rospy.loginfo(msg.msg)
		elif msg.level == msg.WARN:
			rospy.logwarn(msg.msg)
		elif msg.level == msg.ERROR:
			rospy.logerr(msg.msg)
		elif msg.level == msg.FATAL:
			rospy.logfatal(msg.msg)
		else:
			rospy.logwarn(f"Unknown message type received: {msg.msg}")


if __name__ == '__main__':
	rospy.init_node(NODE_NAME)
	rl = ROSLogger()
	rospy.spin()