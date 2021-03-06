#!/bin/sh

# This scripts installs ROS on an OpenNAO distribution
# using the openrobots repository

# Define a constant for the collor
yellow='\e[1;33m'

echo -e "${yellow}==============================================="
echo -e "${yellow}  INSTRUCTIONS: You are about to install ROS"
echo -e "${yellow}  on a fresh OpenNAO distribution. There will"
echo -e "${yellow}  be several instruction blocks like this one."
echo -e "${yellow}  Please read each ENTIRE instruction block "
echo -e "${yellow}  before proceeding (unlike Alex)."
echo -e "${yellow}==============================================="
echo -e "\033[0m"

echo -e "${yellow}==============================================="
echo -e "${yellow}  INSTRUCTIONS: Enter the superuser password "
echo -e "${yellow}  (root) and then nao user password (nao)."
echo -e "${yellow}==============================================="
echo -e "\033[0m"

# Add yourself to the group of sudo-ers
su -c"echo 'nao     ALL=(ALL) ALL' >> /etc/sudoers"
# Check if authentication was not successfull
if [[ $? -ne 0 ]] ; then
    exit 1
fi


# Download a boostraping script which downloads some essential tools
curl -k -s https://chili-research.epfl.ch/ros4nao/bootstrap.sh | sh

# Remove older libyaml if present
if [ -e "/usr/lib/libyaml-cpp.so.0.2" ]; then
  sudo rm /usr/lib/libyaml-cpp.so.0.2
fi

# Attempt to install packages and write the new portage settings
# files if necessary.
SYS_PACKAGES="log4cxx netifaces pyyaml poco apr apr-util yaml-cpp empy"
sudo emerge --autounmask-write $SYS_PACKAGES

echo -e "${yellow}==============================================="
echo -e "${yellow}  INSTRUCTIONS: Select '-3' and then type 'y'"
echo -e "${yellow}  for every prompt after that."
echo -e "${yellow}==============================================="
echo -e "\033[0m"

# Update the portage settings file
sudo etc-update

# Finally install the packages
sudo emerge -Gk $SYS_PACKAGES

# Install nao-robot from the openrobots repo
sudo /opt/openrobots/bin/robotpkgin install nao-robot

# Install ros-comm from the openrobots repo, which is
# currently a broken dependency
sudo /opt/openrobots/bin/robotpkgin install ros-comm ros-image-common

# Update actionlib
sudo robotpkgin remove ros-actionlib-1.9.12
sudo robotpkgin install ros-actionlib-1.11.2
# Install additional ROS packages
sudo robotpkgin install ros-geometry-1.11.3
sudo robotpkgin install ros-image-common-1.11.3
sudo robotpkgin install ros-vision-opencv-1.11.4
sudo robotpkgin install ros-executive-smach-2.0.0

# Set the ROS setup.sh to be called automatically at logon
echo "source /opt/openrobots/etc/ros/setup.sh" >> ~/.bash_profile

echo -e "${yellow}==============================================="
echo -e "${yellow} INSTRUCTIONS: Add you hostname to /etc/hosts"
echo -e "${yellow} and restart your computer to start using ROS."
echo -e "${yellow}==============================================="
echo -e "\033[0m"
