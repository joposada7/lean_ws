#!/usr/bin/env sh

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# set ROS network
export ROS_IP=$(hostname -I | cut -d' ' -f 1)
export ROS_MASTER_URI=http://192.168.50.118:11311

# set _CATKIN_SETUP_DIR
_CATKIN_SETUP_DIR=/home/bean/lean_ws/devel

# source setup.sh from devel
# . /opt/ros/noetic/setup.sh
. ~/lean_ws/devel/setup.sh

exec "$@"
