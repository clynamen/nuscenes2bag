FROM ros:melodic-perception
RUN mkdir ws/src -p
RUN . /opt/ros/melodic/setup.sh && cd ws/src && catkin_init_workspace
COPY .  ws/src
RUN . /opt/ros/melodic/setup.sh && cd ws && catkin_make