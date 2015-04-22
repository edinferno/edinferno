#!/bin/sh

# This scripts installs ROS on an OpenNAO distribution
# using the openrobots repository

# Add yourself to the group of sudo-ers
su -c"echo 'nao     ALL=(ALL) ALL' >> /etc/sudoers"

# Download a boostraping script which downloads some essential tools 
curl -k -s https://chili-research.epfl.ch/ros4nao/bootstrap.sh | sh

# Download binary packages which are system dependencies 
mkdir ros_sys_dep
cd ./ros_sys_dep
wget http://chili-research.epfl.ch/OpenNao/1.14/packages/dev-libs/log4cxx-0.10.0.tbz2
wget http://chili-research.epfl.ch/OpenNao/distfiles/dev-python/netifaces-0.6.tbz2
wget http://chili-research.epfl.ch/OpenNao/distfiles/dev-python/pyyaml-3.09.tbz2
wget http://chili-research.epfl.ch/OpenNao/distfiles/dev-libs/poco-1.3.6_p2.tbz2
wget http://chili-research.epfl.ch/OpenNao/distfiles/dev-libs/apr-1.4.5.tbz2
wget http://chili-research.epfl.ch/OpenNao/distfiles/dev-libs/apr-util-1.3.12.tbz2
 
export PKGDIR=/home/nao/ros_sys_dep/

# Set changes
sudo emerge log4cxx netifaces pyyaml poco apr apr-util
# Apply changes
sudo emerge log4cxx netifaces pyyaml poco apr apr-util

# Remove the downloaded binaries
unset PKGDIR
rm -rf ./ros_sys_dep

# Install nao-robot from the openrobots repo
sudo /opt/openrobots/bin/robotpkgin install nao-robot

# Install ros-comm from the openrobots repo, which is 
# currently a broken dependency
sudo /opt/openrobots/bin/robotpkgin install ros-comm
 
source /opt/openrobots/etc/ros/setup.sh
 
roscore