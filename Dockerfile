FROM osrf/ros:melodic-desktop-full

ARG user=roomac

RUN useradd -ms /bin/bash ${user} && \
    echo "$user:$user" | chpasswd && \
    adduser ${user} sudo && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${user}
WORKDIR /home/${user}

# upgrade because melodic docker wasn't updated in quite long time
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y python-catkin-tools && \
    mkdir -p catkin_ws/src && \
    sudo chown -R ${user}:${user} catkin_ws

COPY --chown=${user}:${user} ./ /home/$user/catkin_ws/src/roomac_ros

WORKDIR /home/${user}/catkin_ws

# update necessary - recently docker wasn't building without it 
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin config --init --extend /opt/ros/melodic && \
    catkin build

WORKDIR /home/${user}

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${user}/.bashrc && \
    echo "source /home/$user/catkin_ws/devel/setup.bash" >> /home/${user}/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/roomac/catkin_ws/src/roomac/roomac_simulation/models/" >> /home/${user}/.bashrc