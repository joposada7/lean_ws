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
		rospy.loginfo("SETTING UP JOYSTICK...")
		self.motors = MotorHandler()
		rospy.on_shutdown(self.motors.cleanup_motors) # Stop motors on node shutdown

		# Get parameters
		self.KILL_SWITCH = rospy.get_param("joy_kill_switch")
		self.DEADZONE = rospy.get_param("joy_deadzone")
		self.LOW_THROTTLE = rospy.get_param("low_throttle")
		self.HIGH_THROTTLE = rospy.get_param("high_throttle")

		# Begin listening to joystick
		self.joy_sub = rospy.Subscriber("joy", Joy, self.spin_motors, queue_size=1)
		rospy.loginfo("JOYSTICK CONTROL READY!")

		# Publish duty cycles
		r = rospy.Rate(360)
		while not rospy.is_shutdown():
			self.motors.publish_duty_cycles()
			r.sleep()

	def spin_motors(self, msg):
		"""
		Handle joystick command by issuing desired duty cycles to each motor.
			- Override command with A button (0) for testing.
			- Horizontal and vertical movements are decoupled.
		"""

		### A BUTTON OVERRIDE ###

		if msg.buttons[0]:
			self.button_override()
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
				self.forward_or_back(linear)
			elif linear == 0.0:
				# Turning in place
				self.turn_in_place(angular)
			else:
				# Move while turning
				self.move_while_turning(linear, angular)

		### HANDLE VERTICAL MOVEMENT ###

		if vertical == 0.0:
			# No vertical movement
			self.motors.LVM.stop_motor()
			self.motors.RVM.stop_motor()
		else:
			# Up or down
			self.up_or_down(vertical)

	##### MOTION PRIMITIVES #####

	def forward_or_back(self, duty_cycle):
		"""
		Move forward or back with LWM/RWM.
		"""
		self.motors.LWM.change_duty_cycle(duty_cycle)
		self.motors.RWM.change_duty_cycle(duty_cycle)
		rospy.loginfo(f"MOVE {round(duty_cycle,1)}%")

	def turn_in_place(self, duty_cycle):
		"""
		Turn in place with LWM/RWM.
		"""
		self.motors.LWM.change_duty_cycle(duty_cycle)
		self.motors.RWM.change_duty_cycle(-duty_cycle)
		rospy.loginfo(f"TURN {round(duty_cycle,1)}%")

	def move_while_turning(self, duty_cycle_linear, duty_cycle_angular):
		scale = self.scale_angular(duty_cycle_angular)
		if duty_cycle_angular > 0.0:
			# Left turns
			self.motors.LWM.change_duty_cycle(duty_cycle_linear)
			self.motors.RWM.change_duty_cycle(scale*duty_cycle_linear)
			rospy.loginfo(f"LEFT CURVE {round(duty_cycle_linear,1)}%")
		else:
			# Right turns
			self.motors.LWM.change_duty_cycle(scale*duty_cycle_linear)
			self.motors.RWM.change_duty_cycle(duty_cycle_linear)
			rospy.loginfo(f"RIGHT CURVE {round(duty_cycle_linear,1)}%")

	def up_or_down(self, duty_cycle):
		"""
		Move up or down with LVM/RVM.
		"""
		self.motors.LVM.change_duty_cycle(duty_cycle)
		self.motors.RVM.change_duty_cycle(duty_cycle)
		rospy.loginfo(f"VERTICAL {round(duty_cycle,1)}%")

	def button_override(self):
		"""
		Do something else if the A button is pressed!
		"""
		DUTY_CYCLE = 50.0
		self.forward_or_back(DUTY_CYCLE)

	##### THROTTLE HANDLERS #####

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
