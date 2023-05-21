#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

from utils.motor_handler import MotorHandler

class JoyHandler():
	"""
	Takes joystick input and converts it to duty cycles for motors.
	Default controls:
		NOTE: If a kill-switch is set, need to have kill-switch pressed down to move!
		Forward/back: LS up/down
		Rotate: 	  RS left/right
		Elevate: 	  RS up/down
	"""
	def __init__(self):
		self.motors = MotorHandler()

		# Get parameters
		self.KILL_SWITCH = rospy.get_param("joy_kill_switch")
		self.DEADZONE = rospy.get_param("joy_deadzone")
		self.LOW_THROTTLE = rospy.get_param("low_throttle")
		self.HIGH_THROTTLE = rospy.get_param("high_throttle")

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
			self.motors.LWM.change_duty_cycle(DUTY_CYCLE)
			self.motors.RWM.change_duty_cycle(DUTY_CYCLE)
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
			self.motors.stop_motors()
			return

		### HANDLE MOVEMENT IN HORIZONTAL PLANE ###

		if linear == 0.0 and angular == 0.0:
			# No horizontal movement
			self.motors.LWM.stop_motor()
			self.motors.RWM.stop_motor()
		else:
			if angular == 0.0:
				# Forward or back
				self.motors.LWM.change_duty_cycle(linear)
				self.motors.RWM.change_duty_cycle(linear)
			elif linear == 0.0:
				# Turning in place
				self.motors.LWM.change_duty_cycle(angular)
				self.motors.RWM.change_duty_cycle(-angular)

			### BLEND HORIZONTAL INPUTS ###

			else:
				scale = self.scale_angular(angular)
				if angular > 0.0:
					# Left turns
					self.motors.LWM.change_duty_cycle(linear)
					self.motors.RWM.change_duty_cycle(scale*linear)
				else:
					# Right turns
					self.motors.LWM.change_duty_cycle(scale*linear)
					self.motors.RWM.change_duty_cycle(linear)

		### HANDLE VERTICAL MOVEMENT ###

		self.motors.LVM.change_duty_cycle(vertical)
		self.motors.RVM.change_duty_cycle(vertical)

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


if __name__ == '__main__':
	rospy.init_node("joy_handler")
	jh = JoyHandler()
	rospy.on_shutdown(jh.motors.cleanup_motors) # Stop motors on node shutdown

	r = rospy.Rate(360)
	while not rospy.is_shutdown():
		jh.motors.publish_duty_cycles()
		r.sleep()
