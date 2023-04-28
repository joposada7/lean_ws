# NoeticLean

ROS1 Noetic workspace for low-power robotic platforms.

LEAN SSID pass: leanpass

LEAN router login: admin; ceimp

Debian Buster ISO: https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/2021-05-07-raspios-buster-arm64.zip

Debian Buster Install Instructions: https://qengineering.eu/install-64-os-on-raspberry-pi-zero-2.html

ROS Noetic Install Instructions: http://wiki.ros.org/noetic/Installation/Debian

# ROS2-Wayfarer

ROS2 communication framework for low-power autonomous testbeds.

# Getting the right OS

## Host computer

For both the Raspberry Pi Zero 2 and the host computer virtual machine, we will be running Ubuntu Jammy Jellyfish (22.04).

You may use a virtualization software of your choice (VMware, VirtualBox). Set up the virtual machine with the desktop image from this link: https://releases.ubuntu.com/jammy/

IMPORTANT: make sure that the network adapter type is set to bridged for your VM. 

## Raspberry Pi

Install the Raspberry Pi Imager from this link: https://www.raspberrypi.com/software/ 

Open the imager, and choose Ubuntu Server 22.04.2 LTS (64-bit) under "Other general-purpose OS" as your operating system. Open the advanced options by pressing the gear at the bottom right and enable SSH, setting a personal username and password. Then configure wireless LAN to the following - SSID: LEAN, Password: wayfarer - and save the options. Flash your microSD card and wait for it to complete.

You can now boot up and SSH into your Pi.

# Getting ROS2

For the most part, the installation for the host computer and the Pi is the same, they will differ in the final installed product, however. 

## Set locale

```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

## Setup sources

You will need to add the ROS2 repository to your system.

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Install ROS2 packages

Update your apt repository caches after setting up the repositories.

```
sudo apt update
```

### Host computer installation

We will do the full desktop install for our host computer (ROS, RViz, demos, tutorials).

```
sudo apt install ros-humble-desktop
```

### Raspberry Pi installation

We will do a bare bones installation for the Pi (Communication libraries, message packages, command line tools. No GUI tools).

```
sudo apt install ros-humble-ros-base
```

## Environment setup

Source the setup script; this makes our ROS2 packages available system wide. For now, we will need to run this whenever we open a new terminal.

```
source /opt/ros/humble/setup.bash
```

## Using this repository

We can now clone our repository (on both host and Pi):

```
git clone https://github.com/alexrice236/ROS2-Wayfarer.git
```

After cloning, we will build our custom packages using colcon.

```
cd ~/ROS2-Wayfarer
colcon build
```

Once the build is finished, open a new terminal and source the underlay:

```
source /opt/ros/humble/setup.bash
```

Go into the root of our workspace:

```
cd ~/ROS2-Wayfarer
```

In the root, source our overlay:

```
source install/local_setup.bash
```

We will now be able to access our launch files and respective packages!