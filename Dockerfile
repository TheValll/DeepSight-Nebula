FROM osrf/ros:humble-desktop

ARG UID=1000
ARG GID=1000
ARG USERNAME=val

RUN apt-get update && apt-get install -y \
    ros-humble-urdf-tutorial \
    ros-humble-gazebo-ros-pkgs \
    sudo \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g $GID $USERNAME \
 && useradd -m -u $UID -g $GID -s /bin/bash $USERNAME \
 && usermod -aG video,dialout,plugdev,sudo $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]