#!/usr/bin/env python3
import rospy
import time
from utils.motor_handler import MotorHandler

if __name__ == '__main__':
	rospy.init_node("pwm_tester")
	motors = MotorHandler()
	rospy.on_shutdown(motors.cleanup_motors)

	DUTY_CYCLE = 100
	rospy.loginfo(f"RUNNING AT {DUTY_CYCLE}%")
	motors.LWM.change_duty_cycle(DUTY_CYCLE)
	motors.RWM.change_duty_cycle(DUTY_CYCLE)

	while True:
		time.sleep(1)

	motors.cleanup_motors()
	