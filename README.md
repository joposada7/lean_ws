
# NoeticLean

ROS1 Noetic workspace for low-power robotic platforms.

Contains ROS OptiTrack Motive Client from https://github.com/mit-aera/OptiTrack-Motive-2-Client.

For access to the router in the LEAN laboratory, connect to:
* SSID: LEAN or LEAN-5G
* pass: leanpass

Access the router admin site via default gateway 192.168.1.50 with credentials:
* user: admin
* pass: ceimp

To login to the LEAN lab laptop, use:
* user: lean-mit
* pass: g0_lean!

The LEAN lab laptop is setup so that SSH to each Pi is passwordless, and using `sudo` on each Pi is also passwordless. Additionally, the network is set up with static IPs so connecting to each robot can be done as follows:

```bash
ssh shrimp # car
ssh wayfarer # boat
ssh blimp # blimp
```

Git is setup for each user's personal Github on each Pi, but performing `git` functions may require a password. To avoid this, you can always directly send files from the host computer to the Pi as follows:

```bash
# Run on host machine connected to LEAN network
rsync -av ~/NoeticLean/src/ shrimp@shrimp:~/NoeticLean/src/
```

# About this repository

This workspace implements previously developed ROS packages from MIT AERA (`optitrack_motive_2_client` from https://github.com/mit-aera/OptiTrack-Motive-2-Client) and MIT ACL (`acl_msgs` from https://github.com/mit-acl), which are used to interface with the motion capture lab at MIT.

As a catkin workspace, please do not commit any `/build` or `/devel` directories, or any files/directories listed in the `.gitignore` file at the root of the repo. These will vary per machine and can always be built using `catkin_make`.

# Getting the right OS

The LEAN lab laptop and the three Pi's in lab have already been configured with the correct OS. Note that the laptop complains about missing a hard drive upon restart, but works fine on shutdown. Not sure why.

ROS Noetic officially supports Ubuntu 20.04 Focal Fossa (Host computer) and Debian Buster (Raspberry Pi). Below are instructions on how to install these on your machine and on the Pi Zero 2 W.

## Host computer

Installation will require a USB drive, and a decent amount of storage on the machine (at least 25G, but more is preferable).

> :warning: **Installation of another OS on the host computer varies by machine, and it's always best to exercise caution as messing up the partitions or  installation can lead to deleting all data on the computer.**

A full-length tutorial for this can be found here: https://www.youtube.com/watch?v=-iSAyiicyQY.

### Partitioning a drive

This step will prepare space on either a HDD or an SDD on your machine to install Ubuntu. You can also follow these instructions: https://www.youtube.com/watch?v=9gS5SoogltE.

On Windows, go to the Start Menu and look for 'disk partition'. Here, you will see all disks installed on your machine and their partitions.

From here, select the drive and partition you'd like to partition to make space for Ubuntu, select 'Shrink Volume' and enter the amount of space you'd like to allocate. Do not convert this space to a Simple Volume.

### Creating a bootable drive

Download the Desktop LTS version of Focal here: https://releases.ubuntu.com/focal/.
Download Rufus here: https://rufus.ie/en/.

In the Rufus software, select the USB drive and the .iso Ubuntu image, then select START. This will delete all data on the USB and turn it into a bootable drive.

### Installing Ubuntu

Keep the drive inserted into the machine, and restart the computer. During boot-up, continually press F12 to enter the boot menu. Note that on some computers, the designated key may be F2, F5, F8, F9, or another key.

In the boot menu, look for 'Boot from USB' or 'Boot Ubuntu'. Some drives may show as their brand (e.g. as SanDisk). This option may be in advanced or boot settings. Once Ubuntu boots, click 'Install Ubuntu' and proceed with the installation. Typically, it's best to go with the 'Minimal Installation' option to preserve space.

On the 'Installation type' window, select 'Something else' in order to select the partition you previously made. You should see it listed as 'free space' under the correct drive.

> :warning: **Be careful on this step! Selecting the wrong drive or partition WILL format it and delete all data you had there, including any OS booting from there.**

Click the + button and create a Logical partition as an Ext4 journaling file system mounted at the root directory `/`, but leave about 4-8GB. Use the final space to create a Logical partition as swap area. Finally, **select the Device for boot loader installation as the partition you create for the root directory `/`**.

From here, you can proceed with the installation as normal. Upon full installation, you will be able to restart the machine, remove the USB drive, and see both your original OS and Ubuntu 20.04 in the GRUB bootloader menu.

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
```bash
sudo apt-get update --allow-releaseinfo-change
sudo apt-get update
sudo apt-get remove --purge vlc
sudo apt-get autoremove
sudo apt-get clean
sudo apt-get upgrade
```

We will now reconfigure our locale:
```bash
sudo raspi-config
```

Choose Localisation Options --> Locale --> All locales --> en_GB.UTF-8 --> Finish

We will now increase the swap file size.

```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
```

Edit the swap file size, increase CONF_SWAPSIZE to 2048, save and exit.

Create, initialize, and start the swap file.
```bash
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

The Raspberry Pi is now ready to install ROS.

# Getting ROS Noetic

Instructions for installing ROS Noetic on both the host machine and the Pi can be found here: 
* Host machine (Ubuntu): http://wiki.ros.org/noetic/Installation/Ubuntu
* Raspberry Pi (Debian): http://wiki.ros.org/noetic/Installation/Debian

## Ubuntu

### Editing permissions

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." 
```bash
add-apt-repository universe
add-apt-repository multiverse
add-apt-repository restricted
```

### Adding the package sources and keys
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

### ROS installation and setup
```bash
sudo apt install ros-noetic-desktop-full
```
When that finishes installing, you can source your underlay:
```bash
source /opt/ros/noetic/setup.bash
```

And test the installation by viewing available packages:
```bash
rospack list-names
```
You should see a list of about 30 available ROS packages.

If you don't want to source the underlay every time you open a terminal, use the following to run automatically on startup.
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Debian

### Editing sources.list

You will need to edit the Debian repository permissions.
```bash
sudo nano /etc/apt/sources.list
```
Uncomment the last three lines in the sources.list file, then save and exit.

### Adding the package sources and keys

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

### ROS installation and setup
We will now install the bare bones Noetic.
```bash
sudo apt install ros-noetic-ros-base
```

When that finishes installing, you can source your underlay:
```bash
source /opt/ros/noetic/setup.bash
```

And test the installation by viewing available packages:
```bash
rospack list-names
```
You should see a list of about 30 available ROS packages.

If you don't want to source the underlay every time you open a terminal, use the following to run automatically on startup.
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Using this repository

We can now clone our repository (on both host and Pi):

```bash
git clone git@github.com:joposada7/lean_ws.git
```

Change to our workspace source directory and update .rosinstall:

```bash
cd ~/NoeticLean/src
wstool update
```

Return to the root of our workspace and build our packages (this might take a while on the Pi):

```bash
cd ../
catkin_make
```

Source the overlay:

```bash
source devel/setup.bash
```

You can also add `source ~/NoeticLean/devel/setup.bash` to the `~/.bashrc` file to automatically source the workspace everytime. You will now be able to access our launch files and respective packages!

> :warning: **If you need to commit to this repository, do not add your `build/` and `devel/` directories. These will be ignored by the .gitignore file.**

# Dependencies

If not already installed, you will need the Eigen and Adafruit INA219 libraries.
```bash
sudo apt install libeigen3-dev # Both computer and Pi
```
```bash
pip3 install adafruit-circuitpython-ina219 # Pi only
```
