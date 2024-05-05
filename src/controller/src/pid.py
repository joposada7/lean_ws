#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from utils.diff_drive_kinematics import DiffDriveKinematics
from utils.viz_tools import VisualizationTools

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Joy
from acl_msgs.msg import ViconState
from controller.msg import Torques

import time # FOR DEBUGGING

class PIDController():
	"""
	Receives goal position and uses PID to publish torque commands for differential-drive
	robot to move towards it.
	"""
	def __init__(self):
		t0 = time.time()

		rospy.loginfo("Initializing PID controller...")
		self.vt = VisualizationTools()
		t1 = time.time()

		# PID parameters
		self.kp = rospy.get_param("kp")
		self.ki = rospy.get_param("ki")
		self.kd = rospy.get_param("kd")
		self.speed = rospy.get_param("speed")
		t2 = time.time()

		# Initialize needed errors and publisher
		now = rospy.Time.now()
		self.previous_time = now.secs + (float(now.nsecs)/10e8)
		self.previous_error = 0.0
		self.accumulated_error = 0.0
		self.torque_pub = rospy.Publisher("/torques", Torques, queue_size=1)
		t3 = time.time()

		# Keep track of position in world-frame
		self.position = None
		self.orientation = None
		vicon_sub = rospy.Subscriber("/vicon_throttle", ViconState, self.update_pos)
		t4 = time.time()

		# Get goal position
		self.previous_goal = None
		self.goal = None
		goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal)
		self.goal_flag_reached_pub = rospy.Publisher("/goal_flag_reached", Bool, queue_size=10)
		t5 = time.time()

		# Waypoints list
		# Start at [-3.68398738 -5.18117237  0.65178227]
		# self.waypoints = np.array([
		# 		[-3.20, -5.20, 0.652],
		# 		[-2.70, -4.90, 0.652]
		# 	])

		# Listen to joystick
		self.kill_switch_pressed = False
		joy_sub = rospy.Subscriber("/joy", Joy, self.check_for_kill_switch, queue_size=1)
		t6 = time.time()

		self.ddk = DiffDriveKinematics()
		rospy.loginfo("PID Controller initialized!")

		t7 = time.time()
		# rospy.loginfo(f"""
		# 	Total elapsed time to initialize: 		  {round(t7-t0, 4)} s

		# 	Time to initialize VisualizationTools: 	  {round(t1-t0, 4)} s
		# 	Time to get rospy parameters: 			  {round(t2-t1, 4)} s
		# 	Time to initialize errors and torque pub: {round(t3-t2, 4)} s
		# 	Time to initialize pose and vicon sub: 	  {round(t4-t3, 4)} s
		# 	Time to initialize goals and goal sub: 	  {round(t5-t4, 4)} s
		# 	Time to initialize joy sub: 			  {round(t6-t5, 4)} s
		# 	Time to initialize DiffDriveKinematics:   {round(t7-t6, 4)} s
		# 	""")


	def check_for_kill_switch(self, msg):
		"""
		Check to see if A button is pressed.
		"""
		if msg.buttons[0]:
			if not self.kill_switch_pressed:
				rospy.loginfo("Detected kill switch pressed.")
			self.kill_switch_pressed = True
		else:
			self.kill_switch_pressed = False

	def publish_control_input(self, heading_error):
		"""
		Takes heading error relative to robot and publishes necessary torque via PID.
		"""
		# PROPORTIONAL TERM
		p = self.kp * heading_error

		# INTEGRAL TERM
		self.accumulated_error += heading_error
		i = self.ki * self.accumulated_error
		if abs(i) > 1.5:
			# Limit integrator windup
			i = 1.5 * i/abs(i)
		if (self.previous_error < 0 and heading_error >= 0) or (self.previous_error > 0 and heading_error <= 0):
			# Reset integrator windup
			i = 0.0
			self.accumulated_error = 0.0

		# DERIVATIVE TERM
		now = rospy.Time.now()
		t = now.secs + (float(now.nsecs)/10e8)
		dt = t - self.previous_time
		d = self.kd * (heading_error - self.previous_error)/dt
		self.previous_time = t

		# TODO: We actually want to convert robot rotation rate -> wheel angular velocities -> voltages/duty cycles,
		# but it's 4 am and i need my zzz

		# Convert control input (robot rotation rate, rad/s) directly to duty cycles
		u = p + i + d
		rospy.loginfo(f"p={p} i={i} d={d}")
		rospy.loginfo(f"u={u}")

		MAX_ROTATION_RATE = 3.58 # rad/s (robot limit when turning and moving at +0.5m/s, reasonably?)
		u = max(-MAX_ROTATION_RATE, min(MAX_ROTATION_RATE, u)) # Cap control input

		LWM_LOW = 70
		LWM_HIGH = 100
		RWM_LOW = 70
		RWM_HIGH = 100
		
		trq = Torques()
		if u == 0.0:
			# Forward
			trq.lwm = LWM_HIGH
			trq.rwm = RWM_HIGH
		elif u < 0.0:
			# Forward left
			trq.lwm = LWM_HIGH
			trq.rwm = u*float(RWM_LOW-RWM_HIGH)/MAX_ROTATION_RATE + RWM_HIGH
		else:
			# Forward right
			trq.lwm = u*float(LWM_HIGH-LWM_LOW)/MAX_ROTATION_RATE + LWM_HIGH
			trq.rwm = RWM_HIGH

		trq.lvm=0.0
		trq.rvm=0.0

		rospy.loginfo(f"Publishing torque msg LWM={str(trq.lwm)}% and RWM={str(trq.rwm)}%")
		self.torque_pub.publish(trq)

	def stop(self):
		"""
		Publish no torque command.
		"""
		trq = Torques()
		trq.lwm = 0.0
		trq.rwm = 0.0
		trq.lvm = 0.0
		trq.rvm = 0.0
		self.torque_pub.publish(trq)
		# rospy.loginfo("Controller shut down!")

	def angular_velocity_to_torque(self, omega):
		"""
		Convert between angular velocity and torque using experimentally derived relation.
		"""
		# TODO!!!
		raise NotImplementedError()

	def get_error(self):
		"""
		Return position error as 3D vector and heading error in horizontal plane.
		"""
		if self.position is None or self.goal is None:
			# Either robot position is unknown or goal is not set
			rospy.logwarn("Tried to get error but position or goal is not known.")
			rospy.logwarn(f"position: {str(self.position)}, goal: {str(self.goal)}")
			return None, None

		RADIUS = 0.35 # HARD CODED: What is the best goal radius for each robot?

		dist_to_goal = np.linalg.norm(self.goal - self.position)
		rospy.loginfo(f"Distance to goal: {dist_to_goal} m")

		if True or dist_to_goal <= RADIUS:
			lookahead_point = self.goal
		else:
			rospy.loginfo("Attempting to find lookahead point...")
			lookahead_point = self.get_lookahead_point(R=RADIUS)
			if lookahead_point is None:
				lookahead_point = self.goal

		rospy.loginfo(f"Found lookahead point {lookahead_point}, vizzing")
		self.vt.draw_point("lookahead", lookahead_point) # Visualization in rviz

		pos_error = lookahead_point - self.position
		rospy.loginfo(f"Current position error = {pos_error} m")
		heading_error = -self.get_signed_angle(self.orientation, pos_error)
		rospy.loginfo(f"Current heading error = {np.rad2deg(heading_error)} deg")

		return (pos_error, heading_error)

	def get_signed_angle(self, vec1, vec2):
		"""
		Get the signed angle between two signed 3D vectors. Assumes normal is Z+ [0,0,1].
		"""
		return np.arctan2(np.dot(np.cross(vec1, vec2), [0,0,1]), np.dot(vec1, vec2))

	def get_lookahead_point(self, R):
		"""
		Find lookahead point on given trajectory.

		This is essentially finding the forward point on a line segment connecting the previous waypoint to
		the current waypoint that intersects a circle of radius R centered at the current position of the robot.

		TODO: Change this to do multiple (3+) waypoints instead of just previous and goal. Will end up
		spinning weirdly when we get arbitrarily close to the goal waypoint.

		See https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm.
		"""
		Q = self.position # Center of circle
		P1 = self.previous_goal # Start point of line segment
		P2 = self.goal
		V = P2-P1

		# Set up quadratic equation
		a = V.dot(V)
		b = 2*V.dot(P1-Q)
		c = P1.dot(P1) + Q.dot(Q) - 2*P1.dot(Q) - R**2

		discriminant = b**2 - 4*a*c
		if discriminant < 0:
			# No real solutions, circle doesn't intersect trajectory!
			return None

		# Larger solution to quadratic equation
		t = (-b + discriminant**0.5)/(2*a)
		if not (0 <= t <= 1):
			# Solution doesn't lie on the line segment
			return None
		return P1 + t*V

	def update_pos(self, msg):
		"""
		Update current known position and orientation using Vicon data.
			- msg: acl_msgs/ViconState
		"""
		# t0 = time.time()
		p = msg.pose.position
		self.position = np.array([p.x, p.y, p.z])
		# t1 = time.time()

		if self.previous_goal is None:
			# Set initial position as the zeroth waypoint
			self.previous_goal = self.position
			rospy.loginfo(f"Set the zeroth waypoint as {self.previous_goal}!")
		# t2 = time.time()

		o = msg.pose.orientation
		theta = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
		# t3 = time.time()
		self.orientation = np.array([np.cos(theta), np.sin(theta), 0])
		self.vt.draw_arrow("position", self.position, [o.x, o.y, o.z, o.w])
		# t4 = time.time()

		# rospy.loginfo(f"""
		# 	Total time to update position: 		 {round(t4-t0, 5)} s

		# 	Time to check extract position: 	 {round(t1-t0, 5)} s
		# 	Time to check previous_goal: 		 {round(t2-t1, 5)} s
		# 	Time to convert euler to quaternion: {round(t3-t2, 5)} s
		# 	Time to draw position arrow: 		 {round(t4-t3, 5)} s
		# 	""")

	def update_goal(self, msg):
		"""
		Update current known goal.
			- msg: geometry_msgs/PoseStamped
		"""
		g = msg.pose.position
		self.goal = np.array([g.x, g.y, g.z])
		rospy.loginfo(f"Got a goal {self.goal}!")
		self.vt.draw_point("goal", self.goal, rgb=[0.0,0.9,0.1])


if __name__ == '__main__':
	rospy.init_node("pid_control")
	pid = PIDController()
	rospy.on_shutdown(pid.stop)
	
	# Start control loop
	END_RADIUS = 0.15 # meters
	rate = rospy.Rate(10)

	# Normal loop
	while not rospy.is_shutdown():
		if pid.goal is None:
			# No goal yet
			pid.stop()
			rate.sleep()
			continue

		# There is a goal, start control
		pos_error, heading_error = pid.get_error()
		if pos_error is None or heading_error is None:
			rospy.logwarn("Could not calculate error, sleeping.")
			rate.sleep()
			continue
			
		if np.linalg.norm(pos_error) > END_RADIUS:
			# Haven't reached goal yet
			pid.publish_control_input(heading_error)
		else:
			# Reached goal
			pid.previous_goal = pid.goal
			pid.goal = None
			# pid.stop()
			pid.goal_flag_reached_pub.publish(data=True)
			rospy.loginfo(f"Reached goal!")
		rate.sleep()

	# wp_i = 0
	# pid.goal = pid.waypoints[0]
	# pid.vt.draw_point("waypoint", pid.waypoints[0], rgb=[0.0,0.9,0.1])

	# while not rospy.is_shutdown():
	# 	if not pid.kill_switch_pressed:
	# 		pid.stop()
	# 		rate.sleep()
	# 		continue
	# 	pos_error, heading_error = pid.get_error()

	# 	if wp_i < len(pid.waypoints):
	# 		if np.linalg.norm(pos_error) > END_RADIUS:
	# 			# Haven't reached goal yet
	# 			pid.publish_control_input(heading_error)
	# 		else:
	# 			# Reached goal
	# 			pid.previous_goal = pid.goal
	# 			wp_i += 1
	# 			if wp_i < len(pid.waypoints):
	# 				pid.goal = pid.waypoints[wp_i]
	# 				pid.vt.draw_point("waypoint", pid.goal, rgb=[0.0,0.9,0.1])
	# 			rospy.loginfo(f"REACHED WAYPOINT {wp_i}!")
	# 	else:
	# 		pid.stop()
