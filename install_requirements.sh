#!/bin/bash
# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo apt install -y ros-$ROS_DISTRO-mavros-msgs can-utils net-tools iproute2

python3 -m pip install --upgrade pip
python3 -m pip install -r $SCRIPT_DIR/requirements.txt
