#!/usr/bin/env python3
import rospy

import RPi.GPIO as GPIO

class Motor():
	"""
	Handles motor speed and direction by taking a desired duty cycle
	between -100% and 100%.
	"""
	def __init__(self, pwmPin, dirPin, motor_frequency):
		"""
		Declare motor with 2 pins.
		"""
		# Set pins as output
		GPIO.setup(pwmPin, GPIO.OUT)
		GPIO.setup(dirPin, GPIO.OUT)

		self.pwm = pwm1 = GPIO.PWM(pwmPin, motor_frequency) # SOFTWARE PWM
		self.dirPin = dirPin
		self.duty_cycle = 0.0

		# Default direction and state (no throttle forward)
		GPIO.output(pwmPin, GPIO.LOW)
		GPIO.output(dirPin, GPIO.LOW)
		self.pwm.start(0)

	def change_duty_cycle(self, duty_cycle):
		"""
		Change to a duty cycle between -100 and 100 (%).
		"""
		if duty_cycle < -100 or duty_cycle > 100:
			rospy.logerr(f"Invalid duty cycle of {duty_cycle}%!")
			return

		self.pwm.start(0)
		self.duty_cycle = duty_cycle # For logging

		if duty_cycle == 0.0:
			self.stop_motor()
		elif duty_cycle > 0.0:
			# Spin forwards
			GPIO.output(self.dirPin, GPIO.LOW)
			self.pwm.ChangeDutyCycle(duty_cycle)
		else:
			# Spin backwards
			GPIO.output(self.dirPin, GPIO.HIGH)
			self.pwm.ChangeDutyCycle(100+duty_cycle)

	def get_duty_cycle(self):
		return self.duty_cycle

	def stop_motor(self):
		"""
		Stop this motor.
		"""
		GPIO.output(self.dirPin, GPIO.LOW)
		self.pwm.stop(0)
		self.duty_cycle = 0.0
