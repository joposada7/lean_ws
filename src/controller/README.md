# lean_ws/controller

This package handles control of the motors and control of each robot.

## Dependencies

This package requires both the Raspberry Pi GPIO and joy interfaces, on both the host computer and the Pi.
```bash
sudo pip3 install RPi.GPIO
sudo apt install ros-noetic-joy
```
However, testing and configuration of the joystick can be done using this package:
```bash
sudo apt install jstest-gtk
```

## Motor control

Motors are controlled on the Pi using the `RPi.GPIO` package. Each of our motors has 2-pins. One pin is the 'direction pin' and is either LOW or HIGH, determining its direction. The other pin is the 'PWM pin', which controls the speed at which the motor spins.

If the direction pin is set to LOW, the PWM pin can have a duty cycle between 0% and 100% to determine the speed. If the direction pin is set to HIGH, the PWM pin reverses, so that 100% means no spin, and 0% means max speed.

The motors and these functions are abstracted for each robot in the `src/utils/motor.py` and `src/utils/motor_handler.py` files. Importing and initializing an object of the `MotorHandler` class automatically initializes each motor according to the robot configurations found in the `configs/` directory.

Each of the 4 motors is labeled as:
* Left forward motor: LWM
* Right forward motor: RWM
* Left vertical motor: LVM
* Right vertical motor: RVM

For the car and the boat, which do not have vertical motors, changing those will do nothing.

Setup of each robot's configuration can be found in the `configs/` file. This includes the number to which each motor is connected to on the PCB, and the duty cycle limits for each robot. For example, the blimp currently should not go above 50% duty cycle for any motor, and the car should not go below 50% duty cycle.

Actuating each motor can be done as follows:
```py
import rospy
from time import sleep
from utils.motor_handler import MotorHandler

rospy.init_node("motors")
mh = MotorHandler()
rospy.on_shutdown(mh.cleanup_motors) # Ensure motors are reset in case of shutdown

# Spin forward motors at 50% forward
mh.motors.LWM.change_duty_cycle(50)
mh.motors.RWM.change_duty_cycle(50)
sleep(3)

# Spin vertical motors in reverse
mh.motors.LVM.change_duty_cycle(-100)
mh.motors.RVM.change_duty_cycle(-100)
sleep(3)

# Stop all motors
mh.motors.stop_motors()
```

## Joystick control

The robots may be controlled via joystick. Currently, the code is setup for an Xbox controller, but can be modified for others is necessary. The default controls are:
* Forward/back: Left joystick up/down
* Turn left/right: Right joystick left/right
* Move up/down: Right joystick up/down
* Kill switch disabled by default

> :warning: **When the Pi momentarily loses connection to the router (i.e. a ping spike), it will continue actuating the motors with the last known command, which may lead to accidents. I think this may be an easy fix but I haven't thought of a solution yet.**

Each robot can be run with the following launch file:
```bash
roslaunch controller joy_teleop.launch user:=shrimp ip:=192.168.50.81
```
This **requires** the robot's Pi to be on, and for the host computer to be connected to the LEAN network. ROS will remotely open nodes on the robot via SSH, and so the SSH configuration on each Pi should not be modified. The LEAN lab laptop's IP is set to 192.168.50.84. Each robot's user and IP on the LEAN network is:
* Car
	* user: shrimp
	* ip: 192.168.50.81
* Boat
	* user: wayfarer
	* ip: 192.168.50.105
* Blimp
	* user: blimp
	* ip: 192.168.50.?

Setup of parameters for joystick control can be found in the `params.yaml` file, which includes a kill switch and deadzone for the joystick.

Note that the car currently has its own launch file, which launches a separate joystick file (`src/car_joy.py` instead of `src/joy.py`) in order to compensate for misaligned wheels and offset center of mass. Launch of this file is simply:
```bash
roslaunch controller car_joy_teleop.launch
```

## Robot Control

Automatic control of each robot functions by publishing `Torques` messages on the `/torques` topic. The `robot_controller` node (in the `robot_controller.py` file) subscribes to these messages and converts the given torques to duty cycles for each motor, then actuates each motor accordingly.

> TODO: `torque_to_duty_cycle` function in `robot_controller.py` has not been implemented properly. This will require more testing of motors  and fitting to torque-voltage curve. Currently, the file just uses the published torques as duty cycles.

## PID Control

Currently, only PID control is implemented for each robot, and has only been tested on the car. This requires pose data from the motion capture room using the `optitrack_motive_2_client` package in this workspace.

This PID works by computing a straight-line trajectory between waypoints, then finding a look-ahead point along the trajectory a certain distance from the robot. If no look-ahead point is found, or the robot is close enough to the goal point, the goal point is used instead. Using this point, the robot calculates a heading error, which is used as the error in the feedback loop. The control input can be interpreted as a desired robot rotation rate (rad/s), which can then be converted to torque.

> TODO: `angular_velocity_to_torque` function in `pid.py` has not been implemented properly. This will require more testing of motors  and fitting to torque-omega curve. Currently, the file just uses the control input as torques.

The car utilizes ground robot differential drive kinematics as implemented in `src/utils/diff_drive_kinematics.py` in order to convert desired robot velocity/turning speed into wheel speeds, or vice versa. This will have to be amended for the boat and blimp, which do not operate with wheels.

Errors are found in 3D space to accommodate the blimp. As such, waypoints must be in (or very close to) the same horizontal plane as the car and boat in order for heading error to be calculated correctly.

Waypoints and robot pose are set in the world frame, as established by the motion capture room. Currently, these are hard-coded into `src/pid.py`, but another method should be used to allow for easier waypoint-setting. Additionally, if no waypoints are set, the user can use `src/goal_pub_script.py` to manually publish individual waypoints for the robot as follows:
```bash
roscd controller
cd src
./goal_pub_script.py -3 4.5 1.7 # x y z
```

Gains for the PID controller can be set in the `params.yaml` file. Some notes on setting gains:
* Proportional gain should be set high enough to allow the robot to turn quickly but not overshoot its turning when a waypoint is not directly in front of it.
* Derivative gain helps smoothen turns, but should be set much lower than the P gain.
* Integral gain is important for the car, which can suffer from a steady-state error during turning (e.g. when the differential drive kinematics believe the car is turning, but it really isn't. See <a href="https://drive.google.com/file/d/1wifkmM5X2MfzGOJSbrk9mukH7MdLaquU/view?usp=sharing">this video</a>)

Finally, automatic control of the robot operates with a kill switch on a joystick controller. On the Xbox controller, holding the A button (or button 0) will begin the control loop, and letting go will stop it.

> :warning: **As mentioned above, if the Pi momentarily loses connection with the router, it will continue actuating the motors with the last known command, which can lead to accidents.**

## ROS Logging

ROS does not automatically log output on remotely launched nodes, so `src/roslogger.py` automatically relogs any output from remote nodes, which can be useful for debugging. Note however, that on shutdown, this file will not log shutdown information. The file has an attribute `WHITELIST` that lists nodes from which to log from.

Be careful not to list `/rosout`, `/rosout_agg`, or `/roslogger` in the whitelist, to prevent clutter on the screen.

Additionally, remember to regularly `rosclean purge -y` to clean all the log files stored in ROS. ROS will also give you a warning when these files exceed 1GB.
