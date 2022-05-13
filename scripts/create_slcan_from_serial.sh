#!/bin/bash
# This script creates virtual CAN port using slcan
# You can use it in 2 ways:
# ./scripts/create_slcan_from_serial.sh - use automatic device path search
# ./scripts/create_slcan_from_serial.sh /dev/ttyACMx - use user device path

# 1. Set tty settings
echo "SLCAN creator settings:"
if [ $# -ge 1 ]; then
    DEV_PATH=$1
    echo "- DEV_PATH:" $DEV_PATH "(user specified)"
else
    source get_sniffer_symlink.sh
    DEV_PATH=$DEV_PATH_SYMLINK
    echo "- DEV_PATH:" $DEV_PATH "(auto)"
fi
if [ $# -ge 2 ]; then
    INTERFACE_NAME=$2
    echo "- INTERFACE_NAME:" $INTERFACE_NAME "(user specified)"
else
    INTERFACE_NAME=slcan0
    echo "- INTERFACE_NAME:" $INTERFACE_NAME "(auto)"
fi
if [ -z $DEV_PATH ]; then
    echo "Can't find expected tty device."
    exit 1
fi
if [ ! -c "$DEV_PATH" ]; then
    echo "SLCAN creator ERROR: specified character device path is not exist."
    exit 1
fi
if [[ $(ifconfig | grep $INTERFACE_NAME) ]]; then
    echo "SLCAN creator: specified interface already exist, skip."
    exit 1
fi

# 2. Run daemon slcand from can-utils - link serial interface with a virtual CAN device
# It will get name slcan name base
#   -o              option means open command
#   -s8             option means 1000 Kbit/s CAN bitrate
#   -t hw           option means UART flow control
#   -S $BAUD_RATE   option means uart baud rate
#   $DEV_PATH       position argument means port name
# sudo slcand -o -s8 -t hw -S $BAUD_RATE $DEV_PATH
sudo slcand -o -c -f -s8 -t hw -S 1000000 $DEV_PATH

sudo ip link set up $INTERFACE_NAME

# Setup SocketCAN queue discipline type
# By default it uses pfifo_fast with queue size 10.
# This queue blocks an application when queue is full.
# So, we use pfifo_head_drop. This queueing discipline drops the earliest enqueued
# packet in the case of queue overflow. 
# More about queueing disciplines:
# https://rtime.felk.cvut.cz/can/socketcan-qdisc-final.pdf
sudo tc qdisc add dev $INTERFACE_NAME root handle 1: pfifo_head_drop limit 1000