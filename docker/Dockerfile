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
    sudo apt-get install -y python-catkin-tools python3-setuptools python-pip python3-vcstool && \
    pip install numpy && \
    mkdir -p catkin_ws/src && \
    sudo chown -R ${user}:${user} catkin_ws

COPY --chown=${user}:${user} ./ /home/$user/catkin_ws/src/roomac_ros

# todo add downloading default map

WORKDIR /home/${user}/catkin_ws

# update necessary - recently docker wasn't building without it 
RUN sudo apt-get update && \
    vcs import src < src/roomac_ros/roomac/roomac.repos && \
    vcs import src < src/roomac_ros/roomac/roomac_simulation.repos && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin config --init --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build

WORKDIR /home/${user}

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${user}/.bashrc && \
    echo "source /home/$user/catkin_ws/devel/setup.bash --extend" >> /home/${user}/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/roomac/catkin_ws/src/roomac_ros/roomac_simulation/models/" >> /home/${user}/.bashrc