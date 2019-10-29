#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/ros_test/devel/setup.bash


roslaunch point_cloud_test auto_start_ros.launch

rosbag play -l ../bagfile/2019-09-16-14-33-07.bag
