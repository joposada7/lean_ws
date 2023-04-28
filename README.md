# NoeticLean

ROS1 Noetic workspace for low-power robotic platforms.

LEAN SSID pass: leanpass

LEAN router login: admin; ceimp


# ROS1-Wayfarer

ROS1 communication framework for low-power autonomous testbeds.

# Getting the right OS

ROS Noetic officially supports Ubuntu 20.04 Focal Fossa (Host computer) and Debian Buster (Raspberry Pi).

## Host computer

You can find the Desktop LTS version of Focal here: https://releases.ubuntu.com/focal/

## Raspberry Pi

Install the Raspberry Pi Imager from this link: https://www.raspberrypi.com/software/ 

Download the Debian Buster ISO: https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/2021-05-07-raspios-buster-arm64.zip

Insert your microSD and choose the custom .iso option, then choose the above downloaded image. Open the advanced options by pressing the gear at the bottom right and enable SSH, setting a personal username and password (name of your platform as username, "pass" as the password). Then configure wireless LAN to the following - SSID: LEAN, Password: leanpass - and save the options. Flash your microSD card and wait for it to complete.

Remove your microSD then put it back into your computer so that you can access the files. Find the file named "bcm2710-rpi-3-b.dtb", copy it, then rename the copy to "bcm2710-rpi-zero-2.dtb".

You can now boot up and SSH into your Pi.

Once the Pi is booted up, enter the following commands:
```
sudo apt-get update --allow-releaseinfo-change
sudo apt-get update
sudo apt-get remove --purge vlc
sudo apt-get autoremove
sudo apt-get clean
sudo apt-get upgrade
```

We will now reconfigure our locale:
```
sudo raspi-config
```

Choose Localisation Options --> Locale --> All locales --> en_GB.UTF-8 --> Finish

We will now increase the swap file size.

```
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
```

Edit the swap file size, increase CONF_SWAPSIZE to 2048, save and exit.

Create, initialize, and start the swap file.
```
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```


# Getting ROS Noetic

For the most part, the installation for the host computer and the Pi is the same, they will differ in the final installed product, however. 

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

# References
Debian Buster Install Instructions: https://qengineering.eu/install-64-os-on-raspberry-pi-zero-2.html

ROS Noetic Install Instructions: http://wiki.ros.org/noetic/Installation/Debian