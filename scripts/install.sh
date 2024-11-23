#!/bin/bash
# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

# Exit immediately if a command exits with a non-zero status
set -e

install_dsdl() {
    echo "Installing tools for C++/Python generation from DSDL..."
    pip install yakut
    echo "DSDL tools installed."
}

install_full() {
    echo "Installing full dependencies..."
    sudo apt install -y git \
                        ros-$ROS_DISTRO-mavros-msgs \
                        ros-$ROS_DISTRO-catkin \
                        python3-pip \
                        python3-catkin-tools \
                        can-utils \
                        net-tools \
                        iproute2
    echo "Full tools installed."
}

usage() {
    echo "Usage: $0 [goal]"
    echo "Goals:"
    echo "  --dsdl    - Install DSDL tools"
    echo "  --full    - Install all dependencies (DSDL + additional tools)"
    echo "Example:"
    echo "  $0 --ros"
    exit 1
}

# Main script logic
if [[ $# -ne 1 ]]; then
    usage
fi

goal=$1

case $goal in
    --dsdl)
        install_dsdl
        ;;
    --full)
        install_full
        ;;
    *)
        echo "Invalid goal: $goal"
        usage
        exit 1
        ;;
esac
