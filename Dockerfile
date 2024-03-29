# Start with a ros-noetic-desktop-full installation. If needed, you can change this to use ROS Melodic or ROS2 as 
# well. If you need CUDA capabilities within your robot workspace, have a look at 
# https://github.com/FieldRobotEvent/competition_environment/blob/main/doc/use_gpu_in_docker.md.
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install Ubuntu packages.
RUN apt-get update && \
  apt-get -y --no-install-recommends install \
    curl \
    git \
    python3-pip && \
  rm -rf /var/lib/apt/lists/*

# Install ROS dependencies (this is the software required by your own robot). Extend this list if your robot has more dependencies.
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
  apt-get update && \
  apt-get -y --no-install-recommends install \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-pointgrey-camera-description \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-hector-gazebo-plugins && \
  rm -rf /var/lib/apt/lists/*   

# Remove dependencies we don't need anymore.
RUN apt-get -y remove curl

# Copy the content of the src folder into the container in the folder '/catkin/src'. You can replace this by cloning your workspace from GIT
# using 'RUN git clone https://github.com/my-robot-repository /catkin' if you like, but be sure to put the files in a folder named '/catkin'! 
COPY src /catkin/src

# Install the dependencies of the repository that are listed in the packages.xml files.
RUN apt-get update && \
  rosdep install -y --from-paths /catkin --ignore-src && \
  rm -rf /var/lib/apt/lists/* 

# Catkin_make your package and source the setup.bash
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /catkin && catkin_make"

# Add the setup.bash to your ROS entrypoint (needed by Docker). This automatically sources your workspace in the CMD command below.
RUN sed -i 's|^\(source .*\)|\1 \&\& source /catkin/devel/setup.bash|g' /ros_entrypoint.sh

# Mountable location that contains map data for tasks 2/3 and where we expect pred_map.csv to go (in task 2). You cannot change these
# locations, because we use these exact locations during the event to provide the 'robot_spawner.launch' file and the driving pattern.
VOLUME ["/catkin/src/virtual_maize_field/map"]
VOLUME ["/catkin/src/virtual_maize_field/launch"]

# Setup the ROS master to communicate with the simulation container. 
ENV ROS_MASTER_URI=http://simulation:11311

# Create default task. We override this variable in the docker-compose file, so you can use this variable in the CMD tag.
ENV TASK=navigation

# Launch your robot. The wait command ensures that this launch file waits for the simulation container to start the ROS server. Change
# this line to start your own robot. The ${TASK} variable will either be navigation or mapping, depending on the current task. You can use this 
# variable set the robot task as is done below. Your launch file is responsible for spawning the robot description in the virual_maize_field
# package.
CMD  roslaunch example_robot_brain task_${TASK}.launch --wait --screen; sleep 999999
