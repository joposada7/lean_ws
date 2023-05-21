#!/usr/bin/env python3
import rospy
import time
from utils.motor_handler import MotorHandler

if __name__ == '__main__':
	rospy.init_node("pwm_tester")
	motors = MotorHandler()
	rospy.on_shutdown(motors.cleanup_motors)

	for i in range(0,101):
		rospy.loginfo(f"RUNNING AT {i}%")
		motors.LWM.change_duty_cycle(i)
		motors.RWM.change_duty_cycle(i)
		time.sleep(0.25)

	motors.cleanup_motors()
	