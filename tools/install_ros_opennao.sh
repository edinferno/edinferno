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

# Attempt to install packages and write the new portage settings
# files if necessary.

SYS_PACKAGES="log4cxx netifaces pyyaml poco apr apr-util tinyxml empy"
sudo emerge --autounmask-write $SYS_PACKAGES

echo -e "${yellow}==============================================="
echo -e "${yellow}  INSTRUCTIONS: Select '-3' and then type 'y'"
echo -e "${yellow}  for every prompt after that."
echo -e "${yellow}==============================================="
echo -e "\033[0m"

# Update the portage settings file
sudo etc-update

# Finally install the packages
sudo emerge -k $SYS_PACKAGES

# Install nao-robot from the openrobots repo
sudo /opt/openrobots/bin/robotpkgin install nao-robot

# Install ros-comm from the openrobots repo, which is 
# currently a broken dependency
sudo /opt/openrobots/bin/robotpkgin install ros-comm ros-image-common
 
# Set the ROS setup.sh to be called automatically at logon
echo "source /opt/openrobots/etc/ros/setup.sh" >> ~/.bash_profile

echo -e "${yellow}==============================================="
echo -e "${yellow} INSTRUCTIONS: Add you hostname to /etc/hosts"
echo -e "${yellow} and restart your computer to start using ROS."
echo -e "${yellow}==============================================="
echo -e "\033[0m"
