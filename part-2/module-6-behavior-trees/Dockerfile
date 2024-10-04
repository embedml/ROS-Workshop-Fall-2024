# SFRT jwore@ncsu.edu
FROM ros:foxy
ENV ROS_DISTRO=foxy
ENV DEBIAN_FRONTEND=noninteractive

# INSTALL AN EASY-TO-USE EDITOR
RUN apt-get update
RUN apt-get install -y nano curl ca-certificates curl apt-utils
RUN apt-get update
RUN apt-get install -y ros-$ROS_DISTRO-py-trees ros-$ROS_DISTRO-py-trees-ros ros-$ROS_DISTRO-py-trees-ros-interfaces 
#ros-$ROS_DISTRO-py-trees-ros-tutorials ros-$ROS_DISTRO-py-trees-js
RUN apt-get update
RUN apt-get install -y tmux

# MAKE PYTHON3 THE DEFAULT PYTHON
RUN echo 'alias python=python3' >> ~/.bashrc
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc
RUN echo 'set constantshow' >> ~/.nanorc
RUN echo 'include /usr/share/nano/python.nanorc' >> ~/.nanorc
RUN echo 'echo "----------------------"' >> ~/.bashrc
RUN echo 'echo "WELCOME TO MODULE 6!"' >> ~/.bashrc
RUN echo 'echo "ROS_DISTRO=$ROS_DISTRO"' >> ~/.bashrc
RUN echo 'echo "----------------------"' >> ~/.bashrc
# SET ENTRYPOINT FOR ROS

WORKDIR /root

CMD ["bash"]
