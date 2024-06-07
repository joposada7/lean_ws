# lean_ws

ROS Noetic workspace for low-power robotic platforms in the Low-Energy Autonomy and Navigation research group.

Contains ROS OptiTrack Motive Client from https://github.com/mit-aera/OptiTrack-Motive-2-Client.

Note that in this repo, "host machine" refers to the laptop used to connect to the Pi and the motion capture system.

# Dependencies

You will need these libraries for controller and sensing functionality.
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

# Using this repository

This workspace implements previously developed ROS packages from MIT AERA (`optitrack_motive_2_client` from https://github.com/mit-aera/OptiTrack-Motive-2-Client) and MIT ACL (`acl_msgs` from https://github.com/mit-acl), which are used to interface with the motion capture lab at MIT.

> **Please see the following documentation files in the `/docs` directory. Setting up the proper OS on the host machine and on the Pi can be found in the [os_setup](./docs/os_setup.md) document. Setting up ROS on the host machine and on the Pi can be found in the [ros_setup](./docs/ros_setup.md) document. Setting up the host machine to Pi network can be found in both the [pi_network_setup](./docs/pi_network_setup.md) and [5g_network_setup](./docs/5g_network_setup.md) documents. Finally, running experiments on the robots can be found in the [running_mocap_experiments](./docs/running_mocap_experiments) document.**

As a catkin workspace, please do not commit any `/build` or `/devel` directories, or any files/directories listed in the `.gitignore` file at the root of the repo. These will vary per machine and can always be built using `catkin_make`.

We can now clone our repository (on both host and Pi):
```bash
git clone git@github.com:joposada7/lean_ws.git
```
Note that if you want to avoid tying an SSH key on the Pi to your personal Github account, the repository files can be sent directly from the host machine to the Pi using rsync:
```bash
rsync -av ~/lean_ws/src/ shrimp@shrimp:~/lean_ws/src/ # On host machine
```

Return to the root of our workspace and build our packages:
```bash
cd ~/lean_ws
catkin_make
```

Add the setup bash script to be sourced in every bash terminal, and source it:
```bash
echo "source ~/lean_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```