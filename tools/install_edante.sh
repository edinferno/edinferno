#!/bin/sh

# Create config folders
mkdir -p /home/nao/config/camera
mkdir -p /home/nao/config/world
mkdir -p /home/nao/config/player

# Add empty calibration files
cp ./resources/top.yaml /home/nao/config/camera/
cp ./resources/bottom.yaml /home/nao/config/camera/
cp ./resources/pitch.yaml /home/nao/config/world/
cp ./resources/player.yaml /home/nao/config/player/

# Update autload.ini so that naoqi loads the camera module
cp ./resources/nao_autoload.ini /home/nao/naoqi/preferences/autoload.ini
sudo cp ./resources/sys_autoload.ini /etc/naoqi/autoload.ini

# Add ros service
sudo cp /resources/ros /etc/init.d/
sudo rc-update add ros default

# Update naoqi service
sudo cp ./resources/naoqi /etc/init.d/
