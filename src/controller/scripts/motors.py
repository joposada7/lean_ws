#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# import RPi.GPIO as GPIO
# from time import sleep

# GPIO.setmode(GPIO.BCM)

Motor1PWM = 20
Motor1Dir = 16
# Motor2PWM = 26
# Motor2Dir = 19
# Motor3PWM = 1
# Motor3Dir = 7
Motor4PWM = 5
Motor4Dir = 0

for motor in motors:
    GPIO.setup(motor, GPIO.OUT) #Initializes all pins as output
    print(f'Set {motor} as output')
    GPIO.output(motor, GPIO.LOW) #Sets default direction of motors
    print(f'Set {motor} to low')

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
    pwm1.ChangeDutyCycle(data.linear)
    # pwm2.ChangeDutyCycle(data.linear)
    # pwm3.ChangeDutyCycle(data.linear)
    pwm4.ChangeDutyCycle(data.linear)

    
     
def listener():
   
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller_node', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()