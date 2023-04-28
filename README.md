# NoeticLean

ROS1 Noetic workspace for low-power robotic platforms.

LEAN SSID pass: leanpass

LEAN router login: admin; ceimp

Debian Buster ISO: https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/2021-05-07-raspios-buster-arm64.zip

Debian Buster Install Instructions: https://qengineering.eu/install-64-os-on-raspberry-pi-zero-2.html

ROS Noetic Install Instructions: http://wiki.ros.org/noetic/Installation/Debian

Run the following to build:

'''
{
cd /NoeticLean/src
wstool update
cd ../
catkin_make
}
'''