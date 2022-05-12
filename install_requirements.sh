#!/bin/bash
sudo apt-get install -y ros-$ROS_DISTRO-tf2-ros

python3 -m pip install --upgrade pip
python3 -m pip install cython scipy pyuavcan yakut