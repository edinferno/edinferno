#!/bin/bash
packages=($(ls ./tools/))
white_list=$(printf "%s;" "${packages[@]}")

cd ./../..
catkin_make -DCATKIN_WHITELIST_PACKAGES=$white_list

if [ "$1" == "camera" ]; then
  catkin_make -DCATKIN_WHITELIST_PACKAGES=$1
fi
