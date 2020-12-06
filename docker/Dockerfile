FROM ros:noetic-ros-base

RUN apt update && \
    apt install -y ros-noetic-turtlesim ros-noetic-turtle-tf2 ros-noetic-tf ros-noetic-tf2-tools ros-noetic-tf-conversions ros-noetic-rqt-graph

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin_make'

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/