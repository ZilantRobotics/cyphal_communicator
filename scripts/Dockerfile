# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="Cyphal communicator"

SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/cyphal_communicator

# 1. Install requirements
COPY scripts/install.sh /tmp/install.sh
RUN apt-get update && /tmp/install.sh --full && rm -rf /tmp/*

# 2. Copy the source files
COPY . /catkin_ws/src/cyphal_communicator

# 3. Build ROS
RUN /catkin_ws/src/cyphal_communicator/scripts/catkin_build.sh

CMD echo "main process has been started"                                            &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash    &&  \
    ./scripts/create_slcan.sh -d $DEV_PATH -i slcan0                                &&  \
    source scripts/config.sh                                                        &&  \
    roslaunch cyphal_communicator cyphal_communicator.launch                        &&  \
    echo "container has been finished"
