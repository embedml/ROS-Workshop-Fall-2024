# SFRT jwore@ncsu.edu
FROM ros:humble
ENV ROS_DISTRO humble
ENV DEBIAN_FRONTEND=noninteractive

# INSTALL AN EASY-TO-USE EDITOR
RUN apt update
RUN apt install -y nano \
      curl \
      ca-certificates \
      apt-utils \
      tmux \
      wget \
      less \
      ros-$ROS_DISTRO-dummy-robot-bringup \
      ros-$ROS_DISTRO-foxglove-bridge


# MAKE PYTHON3 THE DEFAULT PYTHON
RUN echo 'set constantshow' >> ~/.nanorc
RUN echo 'alias python=python3; \
source "/opt/ros/$ROS_DISTRO/setup.bash"; \
echo "----------------------"; \
echo "WELCOME TO WORKSHOP 7!"; \
echo "ROS_DISTRO=$ROS_DISTRO"; \
echo "----------------------"' >> ~/.bashrc
# SET ENTRYPOINT FOR ROS

WORKDIR /root

CMD ["bash"]
