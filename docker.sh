#!/bin/bash
docker run -it -d --name XRoboToolkit \
  -w /projects \
  -v /home/zme/lite_arm_manager/docker:/projects \
  -v /dev/bus/usb:/dev/bus/usb \
  --privileged --net=host \
  ros:humble "$@"

