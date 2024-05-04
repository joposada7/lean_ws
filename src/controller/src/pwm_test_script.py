#!/usr/bin/env python3
import rospy
import time
from utils.motor_handler import MotorHandler

if __name__ == '__main__':
	rospy.init_node("pwm_tester")
	motors = MotorHandler()
	rospy.on_shutdown(motors.cleanup_motors)

	motors.LWM.change_duty_cycle(0)
	motors.RWM.change_duty_cycle(100)
	rospy.loginfo(f"LWM {round(motors.LWM.get_duty_cycle(),1)}%")
	rospy.loginfo(f"RWM {round(motors.RWM.get_duty_cycle(),1)}%")

	while True:
		time.sleep(1)

	motors.cleanup_motors()
	