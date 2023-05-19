#!/usr/bin/env sh

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# set ROS network
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://192.168.50.84:11311

# set _CATKIN_SETUP_DIR
_CATKIN_SETUP_DIR=/home/blimp/NoeticLean/devel

# source setup.sh from devel
# . /opt/ros/noetic/setup.sh
. ~/NoeticLean/devel/setup.sh

exec "$@"
