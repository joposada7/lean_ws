# Running Motion Capture Experiments

This document describes how to run key ROS programs launched from the host machine to move the robot and record data, particularly within the motion capture lab.

## Setup

Ensure that the network between the host machine and the Pi is properly configured and passwordless SSH can be established. Also ensure that ROS is installed on both the host machine and the Pi, and this workspace has been built on both.

Ensure that the correct dependencies have been installed on both, repeated here for convenience:
```bash
# On host machine (running Ubuntu 20.04)
sudo apt install libeigen3-dev ros-noetic-joy python3-pip
pip3 install RPi.GPIO

# On Raspberry Pi Zero 2 W (running Debian buster)
sudo apt install libeigen3-dev ros-noetic-tf ros-noetic-joy python3-pip
pip3 install adafruit-circuitpython-ina219
pip3 install adafruit-circuitpython-typing==1.10.1
pip3 install RPi.GPIO
```

Ensure that the `/etc/hosts` and environment scripts have been properly configured as detailed in the [ros_setup](./ros_setup.md) document.

If experiments are being run in the motion capture lab, ensure that the 5GHz network is being used as detailed in the the [5g_network_setup](./5g_network_setup.md) document.

## Motion capture lab setup

The physical and network setup within the motion capture lab is explained within the the [mocap_setup](./mocap_setup.md) document.

Motion capture can then be used by using the OptiTrack client package, launched on the host machine with:
```bash
roslaunch optitrack_motive_2_client optitrack_lean.launch
```

To make sure it worked, you can launch the rviz file in the `controller` package and see if the robot appears:
```bash
roscd controller
cd rviz
rviz -d frames.rviz
```

## Joystick control

Controlling the robot via joystick control does not require motion capture.

First, connect a joystick (e.g. an Xbox 360 Controller) via Bluetooth or USB to the host machine. Note that Bluetooth tends to be very wonky on Ubuntu, and particularly on dual-booted machines.

Ensure that the Pi configuration found in the `/lean_ws/src/controller/configs/` directory is correct for the right robot.

Then launch the joystick control file using:
```bash
roslaunch controller joy_teleop.launch user:=shrimp ip:=192.168.50.81 log:=true
```
or, for the car robot specifically, launch the properly configured control file:
```bash
roslaunch controller car_joy_teleop.launch user:=wayfarer ip:=192.168.50.107 log:=true
```
adjusting the usernames and IPs as necessary.

The default controls are setup for an Xbox controller:
* Forward/back: Left joystick up/down
* Turn left/right: Right joystick left/right
* Move up/down: Right joystick up/down
* Kill switch disabled by default

## Motion Plan

Reference the [motion_planners_3d repo](https://github.mit.edu/soumyas/motion_planners_3d) to install the motion planner package on the Pi.

Ensure that the motion planner configuration found at `/lean_ws/src/controller/configs/motion_planner_config.yaml` is properly set, particularly with the correct filepaths.

Ensure that the OptiTrack client is already launched and is being visualized in rviz as explained in [Motion capture lab setup](#Motion-capture-lab-setup). Then, launch the motion plan stack using:
```bash
roslaunch controller motion_plan.launch user:=wayfarer ip:=192.168.50.107
```
adjusting the username and IP as necessary.

Note that the code currently assumes (as of 6/7/24) the robot is placed at or near the origin within the motion capture room. The code in `/lean_ws/src/controller/src/planner_helper.py` and `/lean_ws/src/controller/src/spawn_map.py` should be adjusted to correct the transform for motion planning and for spawning the map in rviz.

## Recording pose and power data

Pose and power data can be recorded to .csv files using the sensing package, by launching
```bash
roslaunch sensing record_io.launch user:=wayfarer ip:=192.168.50.107
```
adjusting the username and IP as necessary.

Additionally, a non-ROS power recording script is stored in `/lean_ws/src/sensing/scripts/record_power.py` that can be run directly to record power only. Note that this script incorrectly records time, and should be corrected!!!