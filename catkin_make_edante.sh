#!/bin/bash
if [ "$#" -eq 1 ]; then
  packages=("$1")
else
  packages=($(ls ./edante/) $(ls ./edante/edante_msgs/))
fi

white_list=$(printf "%s;" "${packages[@]}")

cd ./../..
catkin_make -DCATKIN_WHITELIST_PACKAGES=$white_list
