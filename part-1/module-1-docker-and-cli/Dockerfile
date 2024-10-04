# SFRT jwore@ncsu.edu
FROM ros:humble
ENV ROS_DISTRO=humble

# INSTALL AN EASY-TO-USE EDITOR
RUN apt-get update
RUN apt-get install nano
# MAKE PYTHON3 THE DEFAULT PYTHON
RUN echo 'alias python=python3' >> ~/.bashrc
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc
RUN echo 'set constantshow' >> ~/.nanorc
# SET ENTRYPOINT FOR ROS
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

