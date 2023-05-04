# NoeticLean

ROS1 Noetic workspace for low-power robotic platforms.

Contains ROS OptiTrack Motive Client from https://github.com/mit-aera/OptiTrack-Motive-2-Client

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

### Flashing the microSD

Insert your microSD and choose the custom .iso option, then choose the above downloaded image. 

Open the advanced options by pressing the gear at the bottom right and enable SSH, setting a personal username and password (name of your platform as username, "pass" as the password). Then configure wireless LAN to the following - SSID: LEAN, Password: leanpass - and save the options. 

Flash your microSD card and wait for it to complete.

Remove your microSD then put it back into your computer so that you can access the files. Find the file named "bcm2710-rpi-3-b.dtb", copy it, then rename the copy to "bcm2710-rpi-zero-2.dtb".

You can now boot up and SSH into your Pi.


### Configuring the Raspberry Pi

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

The Raspberry Pi is now ready to install ROS.

# Getting ROS Noetic

## Debian

### Editing sources.list

You will need to edit the Debian repository permissions.
```
sudo nano /etc/apt/sources.list
```
Uncomment the last three lines in the sources.list file, then save and exit.

### Adding the package sources and keys

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

### ROS installation and setup
We will now install the bare bones Noetic.
```
sudo apt install ros-noetic-ros-base
```

When that finishes installing, you can source your underlay:
```
source /opt/ros/noetic/setup.bash
```

And test the installation by viewing available packages:
```
rospack list-names
```
You should see a list of about 30 available ROS packages.

If you don't want to source the underlay every time you open a terminal, use the following to run automatically on startup.
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Ubuntu

### Editing permissions

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." 
```
add-apt-repository universe
add-apt-repository multiverse
add-apt-repository restricted
```

### Adding the package sources and keys
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

### ROS installation and setup
```
sudo apt install ros-noetic-desktop-full
```
When that finishes installing, you can source your underlay:
```
source /opt/ros/noetic/setup.bash
```

And test the installation by viewing available packages:
```
rospack list-names
```
You should see a list of about 30 available ROS packages.

If you don't want to source the underlay every time you open a terminal, use the following to run automatically on startup.
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Using this repository

We can now clone our repository (on both host and Pi):

```
git clone git@github.com:alexrice236/NoeticLean.git
```

Change to our workspace source directory and update .rosinstall:

```
cd ~/NoeticLean/src
wstool update
```

Return to the root of our workspace and build our packages (this might take a while on the Pi):

```
cd ../
catkin_make
```

Source the overlay:

```
source devel/setup.bash
```

We will now be able to access our launch files and respective packages!

***IMPORTANT : If you need to commit to this repository, do not add your build/ and devel/ directories.

# References
Debian Buster Install Instructions: https://qengineering.eu/install-64-os-on-raspberry-pi-zero-2.html

ROS Noetic Install Instructions: http://wiki.ros.org/noetic/Installation/Debian