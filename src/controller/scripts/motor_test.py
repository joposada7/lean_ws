#!/usr/bin/env python3
import RPi.GPIO as GPIO
from time import sleep

THROTTLE = 70 # % of Duty Cycle
REVERSE1 = False
REVERSE4 = False
RUN_FOR = 5 # seconds

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

pwm1 = GPIO.PWM(motors[0], 2000)
pwm2 = GPIO.PWM(motors[2], 2000)
pwm3 = GPIO.PWM(motors[4], 2000)
pwm4 = GPIO.PWM(motors[6], 2000)
print('Set pwms')

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)
pwm4.start(0)

if REVERSE1:
    GPIO.output(Motor1Dir, GPIO.HIGH)
    pwm1.ChangeDutyCycle(100 - THROTTLE)
else:
    GPIO.output(Motor1Dir, GPIO.LOW)
    pwm1.ChangeDutyCycle(THROTTLE)

if REVERSE4:
    GPIO.output(Motor4Dir, GPIO.HIGH)
    pwm4.ChangeDutyCycle(100 - THROTTLE)
else:
    GPIO.output(Motor4Dir, GPIO.LOW)
    pwm4.ChangeDutyCycle(THROTTLE)

sleep(RUN_FOR)

for motor in motors:
    GPIO.output(motor, GPIO.LOW)

pwm1.stop(0)
pwm2.stop(0)
pwm3.stop(0)
pwm4.stop(0)
print("Stopped motors")

