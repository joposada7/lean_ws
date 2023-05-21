#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

from utils.motor import Motor
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

		# Get parameters
		self.KILL_SWITCH = rospy.get_param("joy_kill_switch")
		self.DEADZONE = rospy.get_param("joy_deadzone")
		self.LOW_THROTTLE = rospy.get_param("low_throttle")
		self.HIGH_THROTTLE = rospy.get_param("high_throttle")

		# Initialize duty cycle publishers for IO
		self.LWM_pub = rospy.Publisher("lwm_input", Float64, queue_size=1)
		self.RWM_pub = rospy.Publisher("rwm_input", Float64, queue_size=1)
		self.LVM_pub = rospy.Publisher("lvm_input", Float64, queue_size=1)
		self.RVM_pub = rospy.Publisher("rvm_input", Float64, queue_size=1)

		# Begin listening to joystick
		self.joy_sub = rospy.Subscriber("joy", Joy, self.spin_motors, queue_size=1)
		rospy.loginfo("JOY READY")

	def spin_motors(self, msg):
		"""
		Handle joystick command by issuing desired duty cycles to each motor.
			- Override command with A button (0) for testing.
			- Horizontal and vertical movements are decoupled.
		"""

		### A BUTTON OVERRIDE ###

		if msg.buttons[0]:
			DUTY_CYCLE = 50.0
			self.LWM.change_duty_cycle(DUTY_CYCLE)
			self.RWM.change_duty_cycle(DUTY_CYCLE)
			return

		### GET INPUT ###

		linear = self.get_throttle(msg.axes[1]) # Up/down on LS
		angular = self.get_throttle(msg.axes[3]) # Left/right on RS
		vertical = self.get_throttle(msg.axes[4]) # Up/down on RS

		if self.KILL_SWITCH >= 0:
			ks = msg.buttons[self.KILL_SWITCH]
			if not ks:
				if any(i!=0 for i in [linear, angular, vertical]):
					rospy.logwarn("Input ignored, kill switch not pressed!")
				self.stop_motors()
				return

		if linear == 0.0 and angular == 0.0 and vertical == 0.0:
			# No movement at all
			self.stop_motors()
			return

		### HANDLE MOVEMENT IN HORIZONTAL PLANE ###

		if linear == 0.0 and angular == 0.0:
			# No horizontal movement
			self.LWM.stop_motor()
			self.RWM.stop_motor()
		else:
			if angular == 0.0:
				# Forward or back
				self.LWM.change_duty_cycle(linear)
				self.RWM.change_duty_cycle(linear)
			elif linear == 0.0:
				# Turning in place
				self.LWM.change_duty_cycle(angular)
				self.RWM.change_duty_cycle(-angular)

			### BLEND HORIZONTAL INPUTS ###

			else:
				scale = self.scale_angular(angular)
				if angular > 0.0:
					# Left turns
					self.LWM.change_duty_cycle(linear)
					self.RWM.change_duty_cycle(scale*linear)
				else:
					# Right turns
					self.LWM.change_duty_cycle(scale*linear)
					self.RWM.change_duty_cycle(linear)

		### HANDLE VERTICAL MOVEMENT ###

		self.LVM.change_duty_cycle(vertical)
		self.RVM.change_duty_cycle(vertical)

	def get_throttle(self, axis_measure):
		"""
		Get (signed) throttle from joystick axis measurement based on parameters
		high and low throttle.
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


if __name__ == '__main__':
	rospy.init_node("joy_handler")
	jh = JoyHandler()
	rospy.on_shutdown(jh.cleanup_motors) # Stop motors on node shutdown

	r = rospy.Rate(360)
	while not rospy.is_shutdown():
		jh.publish_duty_cycles()
		r.sleep()
