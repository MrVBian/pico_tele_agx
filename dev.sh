#!/bin/bash
docker run -it -d --name xr \
  -w /projects \
  -v /home/zme/pico_tele_agx/:/projects \
  -v /home/zme/robot_config/system_config/zme_robot/cyclonedds_config:/cyclonedds_config
  -v /dev/bus/usb:/dev/bus/usb \
  --privileged --net=host \
  ros:humble "$@"

