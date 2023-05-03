#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo apt install -y ros-$ROS_DISTRO-mavros-msgs

python3 -m pip install --upgrade pip
python3 -m pip install -r $SCRIPT_DIR/requirements.txt