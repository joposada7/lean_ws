#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
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

		# Get indices as per config
		self.LWM = rospy.get_param("left_wheel_motor")-1
		self.RWM = rospy.get_param("right_wheel_motor")-1
		self.LVM = rospy.get_param("left_vertical_motor")-1
		self.RVM = rospy.get_param("right_vertical_motor")-1

		for motor in self.motors:
			GPIO.setup(motor, GPIO.OUT) # Initializes all pins as output
			rospy.loginfo(f"Set {motor} as output")
			GPIO.output(motor, GPIO.LOW) # Sets default direction of motors
			rospy.loginfo(f"Set {motor} to low")

		# Set SOFTWARE PWMs
		motor_frequency = rospy.get_param("motor_frequency")
		pwm1 = GPIO.PWM(self.motors[0], motor_frequency)
		pwm2 = GPIO.PWM(self.motors[2], motor_frequency)
		pwm3 = GPIO.PWM(self.motors[4], motor_frequency)
		pwm4 = GPIO.PWM(self.motors[6], motor_frequency)

		self.pwms = [pwm1, pwm2, pwm3, pwm4]
		for pwm in self.pwms:
			pwm.start(0)
		print(f"PWMs set to {motor_frequency} Hz")

		# Get parameters
		self.KILL_SWITCH = rospy.get_param("joy_kill_switch")
		self.DEADZONE = rospy.get_param("joy_deadzone")
		self.LOW_THROTTLE = rospy.get_param("low_throttle")
		self.HIGH_THROTTLE = rospy.get_param("high_throttle")

		# Begin listening to joystick
		self.joy_sub = rospy.Subscriber("joy", Joy, self.spin_motors, queue_size=1)
		rospy.loginfo("JOY READY")

		# Publish duty cycles for IO
		self.LWM_pub = rospy.Publisher("lwm_input", Float64, queue_size=1)
		self.RWM_pub = rospy.Publisher("rwm_input", Float64, queue_size=1)
		self.LVM_pub = rospy.Publisher("lvm_input", Float64, queue_size=1)
		self.RVM_pub = rospy.Publisher("rvm_input", Float64, queue_size=1)

	def spin_motors(self, msg):
		if msg.buttons[0]:
			# Button A is pressed, override!
			DUTY_CYCLE = 100.0
			self.forward_or_back(DUTY_CYCLE)

		linear = self.get_throttle(msg.axes[1]) # Up/down on LS
		angular = self.get_throttle(msg.axes[3]) # Left/right on RS
		vertical = self.get_throttle(msg.axes[4]) # Up/down on RS

		if self.KILL_SWITCH >= 0:
			ks = msg.buttons[self.KILL_SWITCH]
			if not ks:
				if any(i!=0 for i in [linear, angular, vertical]):
					rospy.logwarn("Input ignored, kill switch not pressed!")
				self.stop_motors(log=False)
				return

		if linear == 0.0 and angular == 0.0 and vertical == 0.0:
			self.stop_motors(log=False) # Stop!
			return

		for pwm in self.pwms:
			pwm.start(0)

		### HANDLE MOVEMENT IN HORIZONTAL PLANE ###

		if angular == 0.0:
			self.forward_or_back(linear) # Linear only
		elif linear == 0.0:
			self.turn_in_place(angular) # Angular only
		else:
			# Blend inputs to turn while moving
			scale = self.scale_angular(angular)
			if linear > 0.0:
				GPIO.output(self.motors[self.RWM*2+1], GPIO.LOW)
				GPIO.output(self.motors[self.LWM*2+1], GPIO.LOW)

				if angular > 0.0:
					# Turning left moving forwards
					self.pwms[self.RWM].ChangeDutyCycle(scale*linear)
					self.pwms[self.LWM].ChangeDutyCycle(linear)
					rospy.loginfo(f"FORWARD LEFT {round(linear,1)}%")

					self.RWM_pub.publish(data=scale*linear)
					self.LWM_pub.publish(data=linear)
				else:
					# Turning right moving forwards
					self.pwms[self.RWM].ChangeDutyCycle(linear)
					self.pwms[self.LWM].ChangeDutyCycle(scale*linear)
					rospy.loginfo(f"FORWARD RIGHT {round(linear,1)}%")

					self.RWM_pub.publish(data=linear)
					self.LWM_pub.publish(data=scale*linear)
			else:
				GPIO.output(self.motors[self.RWM*2+1], GPIO.HIGH)
				GPIO.output(self.motors[self.LWM*2+1], GPIO.HIGH)

				linear = abs(linear)
				if angular > 0.0:
					# Turning left moving backwards
					self.pwms[0].ChangeDutyCycle(100-linear)
					self.pwms[1].ChangeDutyCycle(100-scale*linear)
					rospy.loginfo(f"BACKWARD LEFT {round(linear,1)}%")

					self.RWM_pub.publish(data=-scale*linear)
					self.LWM_pub.publish(data=-linear)
				else:
					# Turning right moving backwards
					self.pwms[0].ChangeDutyCycle(100-scale*linear)
					self.pwms[1].ChangeDutyCycle(100-linear)
					rospy.loginfo(f"BACKWARD RIGHT {round(linear,1)}%")

					self.RWM_pub.publish(data=-linear)
					self.LWM_pub.publish(data=-scale*linear)

		### HANDLE VERTICAL MOVEMENT ###

		if vertical != 0.0:
			self.up_or_down(vertical)
		else:
			self.LVM_pub.publish(data=0.0)
			self.RVM_pub.publish(data=0.0)

	def forward_or_back(self, throttle):
		"""
		Move only forward or backward.
		"""
		for pwm in self.pwms:
			pwm.start(0)
			
		if throttle > 0.0:
			GPIO.output(self.motors[self.RWM*2+1], GPIO.LOW)
			GPIO.output(self.motors[self.LWM*2+1], GPIO.LOW)
			self.pwms[self.RWM].ChangeDutyCycle(throttle)
			self.pwms[self.LWM].ChangeDutyCycle(throttle)
			rospy.loginfo(f"FORWARD {round(throttle,1)}%")

			self.RWM_pub.publish(data=throttle)
			self.LWM_pub.publish(data=throttle)
		elif throttle < 0.0:
			throttle = abs(throttle)
			GPIO.output(self.motors[self.RWM*2+1], GPIO.HIGH)
			GPIO.output(self.motors[self.LWM*2+1], GPIO.HIGH)
			self.pwms[self.RWM].ChangeDutyCycle(100-throttle)
			self.pwms[self.LWM].ChangeDutyCycle(100-throttle)
			rospy.loginfo(f"BACKWARD {round(throttle,1)}%")

			self.RWM_pub.publish(data=-throttle)
			self.LWM_pub.publish(data=-throttle)

	def turn_in_place(self, throttle):
		"""
		Turn in place only.
		"""
		if throttle < 0.0:
			throttle = abs(throttle)
			GPIO.output(self.motors[self.RWM*2+1], GPIO.LOW)
			GPIO.output(self.motors[self.LWM*2+1], GPIO.HIGH)
			self.pwms[self.RWM].ChangeDutyCycle(throttle)
			self.pwms[self.LWM].ChangeDutyCycle(100-throttle)
			rospy.loginfo(f"TURN RIGHT {round(throttle,1)}%")

			self.RWM_pub.publish(data=throttle)
			self.LWM_pub.publish(data=-throttle)
		elif throttle > 0.0:
			GPIO.output(self.motors[self.RWM*2+1], GPIO.HIGH)
			GPIO.output(self.motors[self.LWM*2+1], GPIO.LOW)
			self.pwms[self.RWM].ChangeDutyCycle(100-throttle)
			self.pwms[self.LWM].ChangeDutyCycle(throttle)
			rospy.loginfo(f"TURN LEFT {round(throttle,1)}%")

			self.RWM_pub.publish(data=-throttle)
			self.LWM_pub.publish(data=throttle)

	def up_or_down(self, throttle):
		"""
		Move only up or down.
		"""
		if throttle > 0.0:
			GPIO.output(self.motors[self.LVM*2+1], GPIO.LOW)
			GPIO.output(self.motors[self.RVM*2+1], GPIO.LOW)
			self.pwms[self.LVM].ChangeDutyCycle(throttle)
			self.pwms[self.RVM].ChangeDutyCycle(throttle)
			rospy.loginfo(f"UP {round(throttle,1)}%")

			self.LVM_pub.publish(data=throttle)
			self.RVM_pub.publish(data=throttle)
		elif throttle < 0.0:
			throttle = abs(throttle)
			GPIO.output(self.motors[self.LVM*2+1], GPIO.HIGH)
			GPIO.output(self.motors[self.RVM*2+1], GPIO.HIGH)
			self.pwms[self.LVM].ChangeDutyCycle(100-throttle)
			self.pwms[self.RVM].ChangeDutyCycle(100-throttle)
			rospy.loginfo(f"DOWN {round(throttle,1)}%")

			self.LVM_pub.publish(data=-throttle)
			self.RVM_pub.publish(data=-throttle)

	def get_throttle(self, axis_measure):
		"""
		Get (signed) throttle from joystick axis measurement.
		"""
		if abs(axis_measure) < self.DEADZONE:
			# No throttle in deadzone
			return 0

		# Map joystick movement to desired throttle range
		m = float(self.HIGH_THROTTLE-self.LOW_THROTTLE)/(1.0-self.DEADZONE)
		if axis_measure > 0:
			scaled = m*(axis_measure - self.DEADZONE) + self.LOW_THROTTLE
			return scaled
		else:
			scaled = m*(axis_measure + self.DEADZONE) - self.LOW_THROTTLE
			return scaled 

	def scale_angular(self, angular):
		"""
		Map angular measurement from usual throttle range to allow for turning while moving.
		"""
		high = 0.50
		low = 1.0
		original_range = self.HIGH_THROTTLE-self.LOW_THROTTLE
		new_range = high-low

		scaled = (abs(angular) - self.LOW_THROTTLE)/original_range
		mapped = low + (scaled * new_range)
		return mapped

	def duty_cycle_to_input(self, duty_cycle, reverse=False):
		"""
		Maps given duty cycle from [-HIGH, -LOW]U[LOW,HIGH] to [-100,100].
		"""
		if duty_cycle == 0.0:
			return 0.0

		original_range = self.HIGH_THROTTLE-self.LOW_THROTTLE
		scaled = (abs(duty_cycle) - self.LOW_THROTTLE)/original_range
		mapped = scaled*100.0

		if reverse:
			return -1*mapped
		else:
			return mapped

	def stop_motors(self, log=True):
		"""
		Stop all motors by setting GPIO to low and stopping PWM.
		"""
		for motor in self.motors:
			GPIO.output(motor, GPIO.LOW)
		for pwm in self.pwms:
			pwm.stop(0)

			self.RWM_pub.publish(data=0.0)
			self.LWM_pub.publish(data=0.0)
			self.LVM_pub.publish(data=0.0)
			self.RVM_pub.publish(data=0.0)
		if log:
			rospy.loginfo("Stopped all motors!")

	def cleanup_motors(self):
		"""
		Stop motors and clean up GPIO.
		"""
		self.stop_motors()
		GPIO.cleanup()
		rospy.loginfo("GPIO cleaned up, shutting down.")


if __name__ == '__main__':
	rospy.init_node("joy_handler")
	jh = JoyHandler()
	rospy.on_shutdown(jh.cleanup_motors) # Stop motors on node shutdown
	rospy.spin()
