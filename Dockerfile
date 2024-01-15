FROM ros:noetic-robot-focal

WORKDIR /root/catkin_ws/src

RUN git clone https://github.com/skarful/robot_to_server.git
RUN git checkout master

WORKDIR /root/catkin_ws
RUN catkin build ros_transmitter

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["roslaunch", "ros_transmitter", "ros_transmitter.launch"]

#Note, this dockerfile only launches the ros package and not the server side python script