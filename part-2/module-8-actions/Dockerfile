# SFRT jwore@ncsu.edu
FROM ros:iron
ENV ROS_DISTRO iron
ENV DEBIAN_FRONTEND=noninteractive

# INSTALL AN EASY-TO-USE EDITOR
RUN apt-get update
RUN apt-get install -y nano curl ca-certificates curl apt-utils tmux wget less
RUN apt-get update
#RUN apt-get install -y --fix-missing ros-$ROS_DISTRO-dummy-robot-bringup



# MAKE PYTHON3 THE DEFAULT PYTHON
RUN echo 'set constantshow' >> ~/.nanorc
RUN echo 'alias python=python3; \
source "/opt/ros/$ROS_DISTRO/setup.bash"; \
echo "----------------------"; \
echo "WELCOME TO WORKSHOP 6!"; \
echo "ROS_DISTRO=$ROS_DISTRO"; \
echo "----------------------"' >> ~/.bashrc
# SET ENTRYPOINT FOR ROS

WORKDIR /root

CMD ["bash"]
