ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="Cyphal communicator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/cyphal_communicator


# 1. Install basic requirements
RUN apt-get update                                                              &&  \
    apt-get upgrade -y                                                          &&  \
    apt-get install -y  git ros-$ROS_DISTRO-catkin python3-pip python3-catkin-tools
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# 2. Install requirements
COPY install_requirements.sh install_requirements.sh
RUN ./install_requirements.sh

# 3. Copy the source files
COPY CMakeLists.txt     CMakeLists.txt
COPY package.xml        package.xml

# 4. Build ROS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd ../../ && git config --global http.sslverify false && catkin build

# 5. Copy other files
COPY scripts/ scripts/
COPY launch/ launch/


CMD echo "main process has been started"                                        &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    source /catkin_ws/devel/setup.bash                                          &&  \
    roslaunch cyphal_communicator cyphal_communicator.launch                    &&  \
    echo "container has been finished"