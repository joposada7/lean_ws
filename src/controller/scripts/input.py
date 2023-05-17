#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

from sshkeyboard import listen_keyboard


class KeyInput():
	"""
	"""
	def __init__(self):
		self.linear_speed_pub = rospy.Publisher("linear_speed", Int32, queue_size=10)
		self.angular_speed_pub = rospy.Publisher("angular_speed", Int32, queue_size=10)
		self.vetical_speed_pub = rospy.Publisher("vertical_speed", In32, queue_size=10)

	def press(key):
	    linear_speed = Int32()
	    angular_speed = Int32()
	    vertical_speed = Int32()

	    if key == "w":
	        linear_speed.data = 1
	    if key == "s":
	        linear_speed.data = -1
	    if key == "d":
	        angular_speed.data = 1
	    if key == "a":
	        angular_speed.data = -1
	    if key == "=":
	        vertical_speed.data = 1
	    if key == "-":
	        vertical_speed.data = -1
	    if key == " ":
	        linear_speed.data = 0
	        angular_speed.data = 0
	        vertical_speed.data = 0

        self.linear_speed_pub.publish(linear_speed)
        self.angular_speed_pub.publish(angular_speed)
        self.vertical_speed_pub.publish(vertical_speed)

if __name__ == '__main__':
	rospy.init_node("key_input")
	ki = KeyInput()
	while True:
		listen_keyboard(on_press=ki.press)
	rospy.spin()
