# SFRT jwore@ncsu.edu
FROM ros:humble
ENV ROS_DISTRO=humble

# INSTALL AN EASY-TO-USE EDITOR
RUN apt-get update
RUN apt-get install -y nano curl 

# MAKE PYTHON3 THE DEFAULT PYTHON
RUN echo 'alias python=python3' >> ~/.bashrc
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc
RUN echo 'set constantshow' >> ~/.nanorc
RUN echo 'include /usr/share/nano/python.nanorc' >> ~/.nanorc
RUN echo 'echo "----------------------"' >> ~/.bashrc
RUN echo 'echo "WELCOME TO MODULE 4!"' >> ~/.bashrc
RUN echo 'echo "----------------------"' >> ~/.bashrc
# SET ENTRYPOINT FOR ROS

WORKDIR /
COPY ./ros_entrypoint.sh /
RUN chmod +x ros_entrypoint.sh
# COPY IN FILES
WORKDIR /root
ADD ./workspace_folder ./workspace_folder
#COPY ./test_with_bagging.launch.py /root
WORKDIR /root/workspace_folder
#RUN colcon build
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; colcon build'

WORKDIR /root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
