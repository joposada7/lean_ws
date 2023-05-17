#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

import RPi.GPIO as GPIO

class JoyHandler():
	"""
	Takes joystick input and converts it to Duty Cycles for the LEAN motors.
	Default controls:
		NOTE: Need to have kill-switch pressed down to move!
		Forward/back: LS up/down
		Rotate: 	  RS left/right
		Elevate: 	  RS up/down
	"""
	def __init__(self):
		GPIO.setmode(GPIO.BCM)
		Motor1PWM = 20
		Motor1Dir = 16
		Motor2PWM = 26
		Motor2Dir = 19
		Motor3PWM = 1
		Motor3Dir = 7
		Motor4PWM = 5
		Motor4Dir = 0

		# Initialize all pins to default
		self.motors = [Motor1PWM, Motor1Dir, Motor2PWM, Motor2Dir, Motor3PWM, Motor3Dir, Motor4PWM, Motor4Dir]
		for motor in self.motors:
		    GPIO.setup(motor, GPIO.OUT) # Initializes all pins as output
		    rospy.loginfo(f"Set {motor} as output")
		    GPIO.output(motor, GPIO.LOW) # Sets default direction of motors
		    rospy.loginfo(f"Set {motor} to low")

		# Set SOFTWARE PWMs
		motor_frequency = rospy.get_param("motor_frequency")
	    self.pwm1 = GPIO.PWM(self.motors[0], motor_frequency)
	    self.pwm2 = GPIO.PWM(self.motors[2], motor_frequency)
		self.pwm3 = GPIO.PWM(self.motors[4], motor_frequency)
		self.pwm4 = GPIO.PWM(self.motors[6], motor_frequency)
		print(f"PWMs set to {motor_frequency} Hz")

		self.pwm1.start(0)
		self.pwm2.start(0)
		self.pwm3.start(0)
		self.pwm4.start(0)

		self.KILL_SWITCH = rospy.get_param("joy_kill_switch")
		self.joy_sub = rospy.Subscriber("joy", Joy, self.spin_motors)

	def spin_motors(self, msg):
		linear = self.get_throttle(msg.axes[1]) # Up/down on LS
		angular = self.get_throttle(msg.axes[3]) # Left/right on RS
		vertical = self.get_throttle(msg.axes[4]) # Up/down on RS

		ks = msg.buttons[self.KILL_SWITCH]
		if not ks:
			if any(i!=0 for i in [linear, angular, vertical]):
				rospy.logwarn("Input ignored, kill switch not pressed!")
			self.stop_motors()
			return

		if linear == 0.0 and angular == 0.0 and vertical == 0.0:
			self.stop_motors() # Stop!
			return

		if angular == 0.0:
			self.forward_or_back(linear) # Linear only
		elif linear == 0.0:
			self.turn_in_place(angular) # Angular only
		else:
			# Blend inputs to turn while moving
			if linear > 0.0:
				if angular > 0.0:
					# Turning left moving forwards
					GPIO.output(self.motors[0], GPIO.LOW)
			        GPIO.output(self.motors[2], GPIO.LOW)
			        self.pwm1.ChangeDutyCycle(linear)
			        self.pwm2.ChangeDutyCycle((1.0-angular)*linear)
			        rospy.loginfo(f"FORWARD LEFT")
		        else:
		        	# Turning right moving forwards
		        	GPIO.output(self.motors[0], GPIO.LOW)
			        GPIO.output(self.motors[2], GPIO.LOW)
			        self.pwm1.ChangeDutyCycle((1.0-angular)*linear)
			        self.pwm2.ChangeDutyCycle(linear)
			        rospy.loginfo(f"FORWARD RIGHT")
			else:
				if angular > 0.0:
					# Turning left moving backwards
					GPIO.output(self.motors[0], GPIO.HIGH)
			        GPIO.output(self.motors[2], GPIO.HIGH)
			        self.pwm1.ChangeDutyCycle(100-linear)
			        self.pwm2.ChangeDutyCycle(100-(1.0-angular)*linear)
			        rospy.loginfo(f"BACKWARD LEFT")
		        else:
		        	# Turning right moving backwards
		        	GPIO.output(self.motors[0], GPIO.HIGH)
			        GPIO.output(self.motors[2], GPIO.HIGH)
			        self.pwm1.ChangeDutyCycle(100-(1.0-angular)*linear)
			        self.pwm2.ChangeDutyCycle(100-linear)
			        rospy.loginfo(f"BACKWARD RIGHT")

		if vertical != 0.0:
			self.up_or_down(vertical)

	def forward_or_back(self, throttle):
		"""
		Move only forward or backward.
		"""
		if throttle > 0.0:
			GPIO.output(self.motors[0], GPIO.LOW)
	        GPIO.output(self.motors[2], GPIO.LOW)
	        self.pwm1.ChangeDutyCycle(throttle)
	        self.pwm2.ChangeDutyCycle(throttle)
	        rospy.loginfo(f"FORWARD {throttle}%")
        elif throttle < 0.0:
        	GPIO.output(self.motors[0], GPIO.HIGH)
            GPIO.output(self.motors[2], GPIO.HIGH)
            self.pwm1.ChangeDutyCycle(100-throttle)
            self.pwm2.ChangeDutyCycle(100-throttle)
            rospy.loginfo(f"BACKWARD {throttle}%")

    def turn_in_place(self, throttle):
    	"""
    	Turn in place only.
    	"""
    	if throttle > 0.0:
			GPIO.output(self.motors[0], GPIO.LOW)
	        GPIO.output(self.motors[2], GPIO.HIGH)
	        self.pwm1.ChangeDutyCycle(throttle)
	        self.pwm2.ChangeDutyCycle(100-throttle)
	        rospy.loginfo(f"TURN LEFT {throttle}%")
        elif throttle < 0.0:
        	GPIO.output(self.motors[0], GPIO.HIGH)
            GPIO.output(self.motors[2], GPIO.LOW)
            self.pwm1.ChangeDutyCycle(100-throttle)
            self.pwm2.ChangeDutyCycle(throttle)
            rospy.loginfo(f"TURN RIGHT {throttle}%")

    def up_or_down(self, throttle):
    	"""
    	Move only up or down.
    	"""
    	if throttle > 0.0:
			GPIO.output(self.motors[4], GPIO.LOW)
	        GPIO.output(self.motors[6], GPIO.LOW)
	        self.pwm1.ChangeDutyCycle(throttle)
	        self.pwm2.ChangeDutyCycle(throttle)
	        rospy.loginfo(f"UP {throttle}%")
        elif throttle < 0.0:
        	GPIO.output(self.motors[4], GPIO.HIGH)
            GPIO.output(self.motors[6], GPIO.HIGH)
            self.pwm1.ChangeDutyCycle(100-throttle)
            self.pwm2.ChangeDutyCycle(100-throttle)
            rospy.loginfo(f"DOWN {throttle}%")

	def get_throttle(self, axis_measure):
		"""
		Get (signed) throttle from joystick axis measurement.
		"""
		return axis_measure*100

	def stop_motors(self):
		"""
		Stop all motors by setting GPIO to low and stopping PWM.
		"""
		for motor in self.motors:
		    GPIO.output(motor, GPIO.LOW)
		    rospy.loginfo(f"Set {motor} to low")
		self.pwm1.stop(0)
		self.pwm2.stop(0)
		self.pwm3.stop(0)
		self.pwm4.stop(0)
		rospy.loginfo("Stopped all motors!")


if __name__ == '__main__':
	rospy.init_node("joy_handler")
	jh = JoyHandler()
	rospy.on_shutdown(jh.stop_motors()) # Stop motors on node shutdown
	rospy.spin()
