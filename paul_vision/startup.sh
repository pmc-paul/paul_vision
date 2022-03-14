#!/bin/bash

s_flag='false'

while getopts 's' flag; do
  case "${flag}" in
    s) s_flag='true' ;;
  esac
done

roslaunch paul_vision vision.launch 

if [ $s_flag = "true" ]
then
    rosbag play -l garde-manger.bag
else
    roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
fi
