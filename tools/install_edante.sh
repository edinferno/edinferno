#!/bin/sh

# Create config folder
mkdir -p /home/nao/config/camera
# Add empty calibration files
cp ./resources/top.yaml /home/nao/config/camera/
cp ./resources/bottom.yaml /home/nao/config/camera/

# Update autload.ini so that naoqi loads the camera module
cp ./resources/autoload.ini /home/nao/naoqi/preferences/

# Add ros service
sudo /resources/ros /etc/init.d/
sudo rc-update add ros default

# Update naoqi service
cp ./resources/naoqi /etc/init.d/
