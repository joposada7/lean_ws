#!/usr/bin/env python3
import rospy
import time
from joy import JoyHandler

if __name__ == '__main__':
	rospy.init_node("pwm_tester")
	jh = JoyHandler()
	for i in range(0,101):
		rospy.loginfo(f"RUNNING AT {i}%")
		jh.forward_or_back(i)
		time.sleep(0.25)
	jh.cleanup_motors()