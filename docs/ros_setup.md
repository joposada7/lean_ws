# ROS Setup

This document details the configuration needed to begin working with ROS after installing ROS on the host machine and on the Pi.

> :shipit: **This document assumes that both the host machine and Pi have installed the appropriate OS' as detailed in [os_setup](./os_setup.md) document, and the host-Pi network has been setup as detailed in the [pi_network_setup](./pi_network_setup.md) document.**

## Installing ROS

ROS Noetic is used in this workspace.

### On the host machine (Ubuntu 20.04)

The instructions to install ROS (`ros-noetic-desktop-full`) on Ubuntu can be found at [https://wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu), but the appropriate steps are replicated here.

First, configure your Ubuntu repositories to allow 'restricted', 'universe', and 'multiverse':
```bash
add-apt-repository universe
add-apt-repository multiverse
add-apt-repository restricted
```

Then add the ROS package source:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Then set up the ROS keys:
```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Finally, we can update and install the full desktop ROS Noetic package:
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Additionally, we will install relevant tools and dependencies:
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

To actually use ROS, the setup bash script must be sourced in every bash terminal used. For convenience, we add the source command to the `.bashrc` file:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

### On the Pi (Debian buster)

The instructions to install ROS (`ros-noetic-ros-base`) on Debian can be found at [https://wiki.ros.org/noetic/Installation/Debian](https://wiki.ros.org/noetic/Installation/Debian), but the appropriate steps are replicated here.

You will need to edit the Debian repository permissions and uncomment the last three lines in the `sources.list` file to include the 'contrib' and 'non-free' repositories.

Then, setup the ROS sources and keys:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

Finally, we can update and install bare bones ROS Noetic:
```bash
sudo apt update
sudo apt install ros-noetic-ros-base
```

To actually use ROS, the setup bash script must be sourced in every bash terminal used. For convenience, we add the source command to the `.bashrc` file:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Building this workspace

After installing ROS, this workspace can be built on the host machine and on the Pi. On the host machine (which has installed `ros-noetic-desktop-full`), simply run `catkin_make` in the root directory.

On the Pi (which has installed `ros-noetic-ros-base`), the `/lean_ws/src/gui/` directory must first be deleted, then you can run `catkin_make` in the root directory. Note that the gui package must be deleted since it requires rviz, which is not installed on the Pi. In the future, this may be improved using some kind of Makefile that determines which packages are built based on the OS, or skips the gui package if its dependencies are not met.

## Setting /etc/hosts

In order to remotely launch nodes using ROS, the `/etc/hosts` file on both machines must be properly configured. Simply edit the file on the host machine to include:
```
192.168.50.81 shrimp
192.168.50.105 wayfarer
192.168.50.36 bean
```
adjusting for any changed IPs or usernames.

And edit the file on each Pi to include:
```
192.168.50.100 myhostuser
```
adjusting the IP to reflect your host IP (found using `hostname -I` on the host machine) and the hostname (found using `hostname` on the host machine).

> :warning: **Note that the host IP will change if you switch between an ethernet interface, 2.4GHz Wifi, or 5GHz WiFi. Pi IP will also change if ethernet or 5GHz WiFi is being used. The IPs in `/etc/hosts` must be adjusted accordingly, or ROS programs will fail without readable error.**

## Setting environment scripts

In order to connect to the ROS master on remotely launched nodes, ROS uses environment setup scripts. In this repo, they are stored in `/lean_ws/src/controller/env/` and are different for each robot because each username is different.

Each roslaunch script that remotely launches nodes on the Pi uses these files to setup the ROS environment by matching the right username.

It is essential to check the appropriate env script and adjust the `ROS_MASTER_URI` to match the host machine IP on the LEAN network as shown with the command `hostname -I`. Additionally, if the robot username is changed, the environment script should be renamed to have the correct username (as `user_env.sh`) and the `_CATKIN_SETUP_DIR` should be adjusted in the script.

In the future, it may be possible to have automatic environment initialization by passing the host IP to the remote environment script and identifying the relevant Pi user.