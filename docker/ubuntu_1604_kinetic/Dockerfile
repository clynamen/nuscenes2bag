FROM ros:kinetic-perception
RUN mkdir ws/src -p
RUN . /opt/ros/kinetic/setup.sh && cd ws/src && catkin_init_workspace
COPY .  ws/src
RUN . /opt/ros/kinetic/setup.sh && cd ws && catkin_make
