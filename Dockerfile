FROM osrf/ros:melodic-desktop-full

ARG user=roomac

RUN useradd -ms /bin/bash ${user} && \
    echo "$user:$user" | chpasswd && \
    adduser ${user} sudo && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${user}
WORKDIR /home/${user}

RUN sudo apt-get update && \
    sudo apt-get install -y python-catkin-tools && \
    mkdir -p catkin_ws/src && \
    sudo chown -R ${user}:${user} catkin_ws

COPY --chown=${user}:${user} . /home/$user/catkin_ws/src/roomac_ros

WORKDIR /home/${user}/catkin_ws

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin config --init --extend /opt/ros/melodic && \
    catkin build

WORKDIR /home/${user}

RUN echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"\nsource \"/home/$user/catkin_ws/devel/setup.bash\"" >> /home/${user}/.bashrc