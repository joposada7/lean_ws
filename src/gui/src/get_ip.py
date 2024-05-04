#!/usr/bin/env python3
import rospy
import socket

if __name__ == "__main__":
	"""
	Get the IP that connects to the Pi.
	"""
	rospy.init_node("get_ip")
	pi_ip = rospy.get_param("~pi_ip")
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.connect((pi_ip, 80))
	rospy.set_param("host_ip", s.getsockname()[0])
	rospy.loginfo("Got ip: " + s.getsockname()[0])