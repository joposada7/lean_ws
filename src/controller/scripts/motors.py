#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

Motor1PWM = 20
Motor1Dir = 16
Motor2PWM = 26
Motor2Dir = 19
Motor3PWM = 1
Motor3Dir = 7
Motor4PWM = 5
Motor4Dir = 0
motors = [Motor1PWM, Motor1Dir, Motor2PWM, Motor2Dir, Motor3PWM, Motor3Dir, Motor4PWM, Motor4Dir]

for motor in motors:
    GPIO.setup(motor, GPIO.OUT) #Initializes all pins as output
    print(f'Set {motor} as output')
    GPIO.output(motor, GPIO.LOW) #Sets default direction of motors
    print(f'Set {motor} to low')
GPIO.output(Motor4Dir, GPIO.HIGH)

pwm1 = GPIO.PWM(motors[0], 100)
pwm2 = GPIO.PWM(motors[2], 100)
pwm3 = GPIO.PWM(motors[4], 100)
pwm4 = GPIO.PWM(motors[6], 100)
print('set pwms')

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)
pwm4.start(0)

def callback(data):
    print(data)
    pwm1.ChangeDutyCycle(10*data.linear.x)
    pwm4.ChangeDutyCycle(10*data.linear.x)
    
def listener():
    rospy.init_node('motor_spinner', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()