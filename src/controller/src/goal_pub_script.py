#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
	"""
	Publish a goal pose relative to the /world frame.
	Use: `./goal_pub_script x y z`
	"""
	rospy.init_node("goal_pose_publisher")
	x = float(sys.argv[1])
	y = float(sys.argv[2])
	z = float(sys.argv[3])

	pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
	msg = PoseStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "/world"

	msg.pose.position.x = x
	msg.pose.position.y = y
	msg.pose.position.z = z

	msg.pose.orientation.x = 0.0
	msg.pose.orientation.y = 0.0
	msg.pose.orientation.z = 0.0
	msg.pose.orientation.w = 1.0

	while pub.get_num_connections() <= 0:
		rospy.sleep(0.01)
	pub.publish(msg)
	rospy.spin()