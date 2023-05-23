#!/usr/bin/env python3
import rospy
from utils.joy_handler import JoyHandler

class CarJoy(JoyHandler):
	"""
	Car-specific joystick controller. Inherits utils/JoyHandler.
	Deals with misaligned wheels!
	"""
	def forward_or_back(self, duty_cycle):
		"""
		Move forward or back with LWM/RWM, compensating for wheel misalignment.
		"""
		lwm, rwm = self.compensate(duty_cycle)
		self.motors.LWM.change_duty_cycle(lwm)
		self.motors.RWM.change_duty_cycle(rwm)
		rospy.loginfo(f"LWM {round(self.motors.LWM.get_duty_cycle(),1)}%")
		rospy.loginfo(f"RWM {round(self.motors.RWM.get_duty_cycle(),1)}%")

	def move_while_turning(self, duty_cycle_linear, duty_cycle_angular):
		lwm, rwm = self.compensate(duty_cycle_linear)
		scale = self.scale_angular(duty_cycle_angular)
		if duty_cycle_angular > 0.0:
			# Left turns
			self.motors.LWM.change_duty_cycle(lwm)
			self.motors.RWM.change_duty_cycle(scale*rwm)
		else:
			# Right turns
			self.motors.LWM.change_duty_cycle(scale*lwm)
			self.motors.RWM.change_duty_cycle(rwm)
		rospy.loginfo(f"LWM {round(self.motors.LWM.get_duty_cycle(),1)}%")
		rospy.loginfo(f"RWM {round(self.motors.RWM.get_duty_cycle(),1)}%")

	def compensate(self, duty_cycle):
		"""
		Compensate for misaligned wheels while moving forward or back.
		Returns compensated (signed) duty cycles for LWM and RWM.

		TODO: Forward left vs. right turns have different radii of rotation,
		since for left turns RWM -> 100% but in right turns, LWM only goes -> 71%.
		Maybe fixable by incorporating angular measurement in this compensation,
		but that's a lot of brain I don't have !
		"""
		if duty_cycle == 0.0:
			return (0.0, 0.0)

		if duty_cycle > 0.0:
			LWM = 0.71*duty_cycle
			RWM = duty_cycle
		else:
			LWM = 0.80*duty_cycle
			RWM = duty_cycle
		return (LWM, RWM)

	def button_override(self):
		"""
		For MOCAP testing:
		https://docs.google.com/spreadsheets/d/1xdeZvM1NdKvcoEhWO4j_LPbAxA-xvcWVDYXAXQ32RLA/edit?usp=sharing
		"""
		DUTY_CYCLE = 80.0
		self.motors.LWM.change_duty_cycle(DUTY_CYCLE)
		self.motors.RWM.change_duty_cycle(DUTY_CYCLE)
		# self.forward_or_back(DUTY_CYCLE)


if __name__ == '__main__':
	rospy.init_node("car_joy")
	cj = CarJoy()
