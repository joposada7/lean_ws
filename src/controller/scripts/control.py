#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

from sshkeyboard import listen_keyboard

class MotorControl():
	"""
	"""
	def __init__(self):
		self.THROTTLE = 15 # DUTY CYCLE

		self.ls_sub = rospy.Subscriber("linear_speed", Int32, self.linear_handle)
		self.as_sub = rospy.Subscriber("angular_speed", Int32, self.angular_handle)
		self.vs_sub = rospy.Subscriber("vertical_speed", Int32, self.vertical_handle)

		self.ls_moving = False
		self.as_moving = False
		self.vs_moving = False

	def linear_handle(self, msg):
		ls = msg.data
		self.ls_moving = True
		if ls > 0.5:
			GPIO.output(Motor1Dir, GPIO.LOW)
            GPIO.output(Motor2Dir, GPIO.LOW)
            pwm1.ChangeDutyCycle(throttle)
            pwm2.ChangeDutyCycle(throttle)
            rospy.loginfo("FORWARD")
		elif ls < -0.5:
			GPIO.output(Motor1Dir, GPIO.HIGH)
            GPIO.output(Motor2Dir, GPIO.HIGH)
            pwm1.ChangeDutyCycle(100-throttle)
            pwm2.ChangeDutyCycle(100-throttle)
            rospy.loginfo("BACK")
        else:
        	self.ls_moving = False

	def angular_handle(self, msg):
		angs = msg.data
		self.as_moving = True
		if angs > 0.5:
            GPIO.output(Motor1Dir, GPIO.LOW)
            GPIO.output(Motor2Dir, GPIO.HIGH)
            pwm1.ChangeDutyCycle(throttle)
            pwm2.ChangeDutyCycle(100-throttle)
            rospy.loginfo("RIGHT")
        elif angs < -0.5:
            GPIO.output(Motor1Dir, GPIO.HIGH)
            GPIO.output(Motor2Dir, GPIO.LOW)
            pwm1.ChangeDutyCycle(100-throttle)
            pwm2.ChangeDutyCycle(throttle)
            rospy.loginfo("LEFT")
        else:
        	self.as_moving = False

	def vertical_handle(self, msg):
		vs = msg.data
		self.vs_moving = True
		if vs > 0.5:
            GPIO.output(Motor3Dir, GPIO.LOW)
            GPIO.output(Motor4Dir, GPIO.LOW)
            pwm3.ChangeDutyCycle(throttle)
            pwm4.ChangeDutyCycle(throttle)
            rospy.loginfo("UP")
        elif vs < -0.5:
            GPIO.output(Motor3Dir, GPIO.HIGH)
            GPIO.output(Motor4Dir, GPIO.HIGH)
            pwm3.ChangeDutyCycle(100-throttle)
            pwm4.ChangeDutyCycle(100-throttle)
            rospy.loginfo("DOWN")
        else:
        	self.vs_moving = True

    	# Handle here only
    	if not self.ls_moving and not self.as_moving and not self.vs_moving:
    		GPIO.output(Motor1Dir, GPIO.LOW)
	        GPIO.output(Motor2Dir, GPIO.LOW)
	        GPIO.output(Motor3Dir, GPIO.LOW)
	        GPIO.output(Motor4Dir, GPIO.LOW)
	        pwm1.ChangeDutyCycle(0)
	        pwm2.ChangeDutyCycle(0)
	        pwm3.ChangeDutyCycle(0)
	        pwm4.ChangeDutyCycle(0)

if __name__ == '__main__':
	rospy.init_node("motor_control")

	Motor1PWM = 20
	Motor1Dir = 16
	Motor2PWM = 26
	Motor2Dir = 19
	Motor3PWM = 1
	Motor3Dir = 7
	Motor4PWM = 5
	Motor4Dir = 0
	motors = [Motor1PWM, Motor1Dir, Motor2PWM, Motor2Dir, Motor3PWM, Motor3Dir, Motor4PWM, Motor4Dir]

	GPIO.setmode(GPIO.BCM)

	for motor in motors:
	    GPIO.setup(motor, GPIO.OUT) #Initializes all pins as output
	    print(f'Set {motor} as output')
	    GPIO.output(motor, GPIO.LOW) #Sets default direction of motors
	    print(f'Set {motor} to low')

	pwm1 = GPIO.PWM(motors[0], 1000) #SOFTWARE
	pwm2 = GPIO.PWM(motors[2], 1000) #SOFTWARE
	pwm3 = GPIO.PWM(motors[4], 1000) #SOFTWARE
	pwm4 = GPIO.PWM(motors[6], 1000) #SOFTWARE

	pwm1.start(0)
	pwm2.start(0)
	pwm3.start(0)
	pwm4.start(0)

	mc = MotorControl()
	try:
		rospy.spin()
	except KeyboardInterrupt:
	    pwm1.stop(0)
	    pwm2.stop(0)
	    pwm3.stop(0)
	    pwm4.stop(0)

	    GPIO.cleanup()