# Example robot workspace
This is an example workspace to show how to setup a robot workspace that works with the simulation environment that will be used at the Field Robot Event.

Your robot workspace should have a launch file that is responsible for:
* Spawning the robot using the `robot_spawner.launch` file in the `virtual_maize_field` package.
* The robots behaviour and all its required nodes.

See the [example launch file](src/example_robot_brain/launch/task_1.launch).

The robot used in the simulation is the Clearpath Jackal, you can find detailed instructions and documentation at http://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html. Be aware that the Jackal comes with a GPS but that the use of a GNSS receiver is not allowed except for the Free Style task. The focus for the other tasks in terms of localisation shall be on relative positioning and sensor based behaviours.

---
**NOTE**

It is important that the `virtual_maize_field` package is present in the `src` directory of your workspace. You can add this package to your own workspace by:
```commandline
git submodule add https://github.com/FieldRobotEvent/virtual_maize_field
```
---

## Install workspace
To use this workspace, clone it to your computer, install the depenencies, build the packages and source the workspace:
```bash
git clone https://github.com/FieldRobotEvent/example_ws --recurse-submodules
rosdep install -y --from-paths ~/example_ws --ignore-src
cd ~example_ws
catkin_make
source ~/example_ws/devel/setup.bash
```

## Launch your robot locally
1. Launch the simulation environment:
```commandline
roslaunch virtual_maize_field simulation.launch
```
2. Launch your robot software:
```commandline
roslaunch example_robot_brain task_1.launch
```

## Build the Docker image of your robot
During the competition, there will be two containers - the simulation container, running rosmaster and gazebo, and the robot container, running your code.
This allows you to have as much freedom as you desire in completing the tasks, without any risk of having mixed dependencies (such as ROS/ROS2) interfering with your solution. To create the robot container, make sure that you installed Docker on our computer. Change the [Dockerfile](Dockerfile) to the requirements of your robot and build the image:

```commandline
docker build . -t robot_workspace
```

After building the image, an image with the name `robot_workspace` should be available in Docker. You can check this by running the `docker images` command.

To test your robot in the Docker environent, clone the [competition environment](https://github.com/FieldRobotEvent/competition_environment) and follow the instructions from there.

To save the docker image as file, run `docker image save robot_workspace | gzip -c - > robot_workspace.tgz`. This can take a few minutes.

## Hints for starting your robot container from scratch
* Be aware of the mounts we are going to use in the docker-compose.yml files found in the [competition environment](https://github.com/FieldRobotEvent/competition_environment). We will be using these mount points (and only these mount points), so make sure you expose those VOLUMEs in your container for us to use. This probably won't look pretty if you're building from scratch, but you will have to deal with it.
* The `simulation_container` runs a rosmaster at `http://simulation:11311`, as stated in the ENV field in the [example DOCKERFILE](Dockerfile). You will need to communicate to this port to interact with gazebo.
* Make sure your CMD is set correctly - we do not plan on overriding this in the docker-compose.yml file we will use in competition mode. 

Beyond that, you are free to completely rebuild the entire environment as you see fit. 

## Trouble shooting
* `Could not open file[/home/.../example_ws/src/virtual_maize_field/worlds/generated.world]` by launching Gazebo: There is no world file generated. Generate a world file by (for example) `rosrun virtual_maize_field create_task_1_mini`.
* `VMware: vmw_ioctl_command error Invalid argument.` by launching Gazebo: Execute `echo "export SVGA_VGPU10=0" >> ~/.profile` in the terminal and reboot your virtual machine. 
* `Error in REST request` when launching gazebo: Open `~/.ignition/fuel/config.yaml` and change the line: `url: https://api.ignitionfuel.org` to `url:  https://api.ignitionrobotics.org`.
* If the lidar data on the topic `front/scan` only returns ranges with the value `inf`, even though in simulation the lidar should ‘see’ certain objects within its range, you have to execute `export LIBGL_ALWAYS_SOFTWARE=1` in the terminal in which you launch gazebo. You have to run this command before starting gazebo. This solves the problem with the lidar, but might have some consequences on the rendering speed of gazebo. 
