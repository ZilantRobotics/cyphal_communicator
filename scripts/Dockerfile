# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="Cyphal communicator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/cyphal_communicator


# 1. Install basic requirements
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y git ros-$ROS_DISTRO-catkin python3-pip python3-catkin-tools

# 2. Install requirements
COPY install_requirements.sh install_requirements.sh
COPY requirements.txt requirements.txt
RUN ./install_requirements.sh


# 3. Copy the source files
COPY CMakeLists.txt     CMakeLists.txt
COPY package.xml        package.xml
COPY src/ src/
COPY compile_dsdl.sh compile_dsdl.sh
COPY Libs/ Libs/
COPY .git/ .git/

# 4. Build ROS
RUN ./Libs/cyphal_application/scripts/nnvg_generate_c_headers.sh
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd ../../ && git config --global http.sslverify false && catkin build

# 5. Copy other files
COPY scripts/ scripts/
COPY launch/ launch/
COPY scripts/config.sh scripts/config.sh


CMD echo "main process has been started"                                            &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash    &&  \
    ./scripts/create_slcan.sh -d $DEV_PATH -i slcan0                                &&  \
    source scripts/config.sh                                                        &&  \
    roslaunch cyphal_communicator cyphal_communicator.launch                        &&  \
    echo "container has been finished"
