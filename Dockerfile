FROM ros:kinetic-ros-base

# sh can't source ros setup script
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Setup the catkin workspace
RUN mkdir -p ~/catkin_ws/src \
    && source /ros_entrypoint.sh \
    && catkin_init_workspace ~/catkin_ws

# Copy the source into the image
ADD . ~/catkin_ws/src

# Install dependencies
WORKDIR ~/catkin_ws
RUN apt-get update && \
    rosdep install --from-paths . --as-root=apt:no --ignore-src --rosdistro=kinetic -y

# Build the workspace
RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic

# Build the workspace
CMD ["roslaunch", "phoenixbot_launch", "phoenixbot_auto2018.launch", "sim:=true", "rviz:=false"]
