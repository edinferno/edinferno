#!/bin/bash
if [ "$#" -eq 1 ]; then
  if [ "$1" == "msgs" ]; then
    packages=($(ls ./edante/edante_msgs/))
  else
    packages=("$1")
  fi
else
  packages=($(ls ./edante/) $(ls ./edante/edante_msgs/))
fi

white_list=$(printf "%s;" "${packages[@]}")

cd ./../..
catkin_make -DCATKIN_WHITELIST_PACKAGES=$white_list
