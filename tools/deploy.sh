#!/bin/sh

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <IP of the robot>" >&2
  exit 1
fi

echo Executing rsync to $1:/home/nao
rsync -razv /home/nao/ros nao@$1:/home/nao

