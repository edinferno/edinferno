#!/bin/bash
packages=($(ls ./edante/))
white_list=$(printf "%s;" "${packages[@]}")

cd ./../..
catkin_make -DCATKIN_WHITELIST_PACKAGES=$white_list
