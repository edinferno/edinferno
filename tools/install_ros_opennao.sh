#!/bin/sh

# This scripts installs ROS on an OpenNAO distribution
# using the openrobots repository

# Add yourself to the group of sudo-ers
su -c"echo 'nao     ALL=(ALL) ALL' >> /etc/sudoers"

# Download a boostraping script which downloads some essential tools 
curl -k -s https://chili-research.epfl.ch/ros4nao/bootstrap.sh | sh

# Set changes
sudo emerge --autounmask-write log4cxx netifaces pyyaml poco apr apr-util

etc-update
# Apply changes
sudo emerge log4cxx netifaces pyyaml poco apr apr-util

# Install nao-robot from the openrobots repo
sudo /opt/openrobots/bin/robotpkgin install nao-robot

# Install ros-comm from the openrobots repo, which is 
# currently a broken dependency
sudo /opt/openrobots/bin/robotpkgin install ros-comm
 
source /opt/openrobots/etc/ros/setup.sh
 
roscore