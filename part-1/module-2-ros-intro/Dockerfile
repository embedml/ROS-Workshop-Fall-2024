# SFRT jwore@ncsu.edu
FROM ros:humble
ENV ROS_DISTRO=humble

# INSTALL AN EASY-TO-USE EDITOR
RUN apt-get update && apt-get install -y nano git tmux


RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'alias python=python3' >> ~/.bashrc
RUN echo 'echo "----------------------"' >> ~/.bashrc
RUN echo 'echo "WELCOME TO MODULE 2!"' >> ~/.bashrc
RUN echo 'echo "----------------------"' >> ~/.bashrc
WORKDIR /root
ADD ./src ./src

CMD ["bash"]
