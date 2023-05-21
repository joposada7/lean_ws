#!/usr/bin/env python3
import rospy
from utils.motor import Motor
from std_msgs.msg import Float64

import RPi.GPIO as GPIO

class MotorHandler():
	"""
	Handles the motors on 4-motor differential drive robot. Relies on config.yaml for motor configuration on robot.
	
	Use:
		motors = MotorHandler()
		motors.LWM.change_duty_cycle(50)
		motors.RWM.change_duty_cycle(-50)
		rospy.on_shutdown(motors.cleanup_motors)

	See motor.py for individual motor use.
	"""
	def __init__(self):
		"""
		Initialize pins and motors.
		"""
		GPIO.setmode(GPIO.BCM)

		# Declare pins
		Motor1PWM = 20
		Motor1Dir = 16
		Motor2PWM = 26
		Motor2Dir = 19
		Motor3PWM = 1
		Motor3Dir = 7
		Motor4PWM = 5
		Motor4Dir = 0

		# Initialize all motors
		motor_frequency = rospy.get_param("motor_frequency")
		pins = [Motor1PWM, Motor1Dir, Motor2PWM, Motor2Dir, Motor3PWM, Motor3Dir, Motor4PWM, Motor4Dir]
		lwm_i = rospy.get_param("left_wheel_motor")-1
		rwm_i = rospy.get_param("right_wheel_motor")-1
		lvm_i = rospy.get_param("left_vertical_motor")-1
		rvm_i = rospy.get_param("right_vertical_motor")-1
		self.LWM = Motor(pins[2*lwm_i], pins[2*lwm_i+1], motor_frequency)
		self.RWM = Motor(pins[2*rwm_i], pins[2*rwm_i+1], motor_frequency)
		self.LVM = Motor(pins[2*lvm_i], pins[2*lvm_i+1], motor_frequency)
		self.RVM = Motor(pins[2*rvm_i], pins[2*rvm_i+1], motor_frequency)
		self.motors = [self.LWM, self.RWM, self.LVM, self.RVM]

		# Initialize duty cycle publishers for IO
		self.LWM_pub = rospy.Publisher("lwm_input", Float64, queue_size=1)
		self.RWM_pub = rospy.Publisher("rwm_input", Float64, queue_size=1)
		self.LVM_pub = rospy.Publisher("lvm_input", Float64, queue_size=1)
		self.RVM_pub = rospy.Publisher("rvm_input", Float64, queue_size=1)

	def stop_motors(self):
		"""
		Stop all motors by setting GPIO to low and stopping PWM.
		"""
		for motor in self.motors:
			motor.stop_motor()

	def cleanup_motors(self):
		"""
		Stop motors and clean up GPIO.
		"""
		self.stop_motors()
		GPIO.cleanup()
		rospy.loginfo("GPIO cleaned up, shutting down.")

	def publish_duty_cycles(self):
		"""
		Publish each motor's current duty cycle.
		"""
		self.LWM_pub.publish(data=self.LWM.get_duty_cycle())
		self.RWM_pub.publish(data=self.RWM.get_duty_cycle())
		self.LVM_pub.publish(data=self.LVM.get_duty_cycle())
		self.RVM_pub.publish(data=self.RVM.get_duty_cycle())
