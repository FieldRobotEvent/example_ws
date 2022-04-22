# Example robot workspace
This is an example workspace to show how to setup a robot workspace that works with the simulation environment that will be used at the Field Robot Event. You are free to implement your own robot, see the rules on the [Field Robot Event website](https://www.fieldrobot.com/event/index.php/contest-hybrid/tasks-h/).

Your robot workspace should have a launch file that is responsible for:
* Spawning the robot using the `robot_spawner.launch` file in the `virtual_maize_field` package.
* The robots behaviour and all its required nodes.

See the [example launch file](src/example_robot_brain/launch/task_navigation.launch).

The robot used in the simulation is the Clearpath Jackal, you can find detailed instructions and documentation at http://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html. Be aware that the Jackal comes with a GPS but that the use of a GNSS receiver is not allowed except for the Free Style task. The focus for the other tasks in terms of localisation shall be on relative positioning and sensor based behaviours.

---
**NOTE**

It is important that the `virtual_maize_field` package is present in the `src` directory of your workspace. You can add this package to your own workspace by:
```commandline
cd src
git submodule add https://github.com/FieldRobotEvent/virtual_maize_field
```
---

## Install workspace
To use this workspace, clone it to your computer, install the depenencies, build the packages and source the workspace:
```bash
git clone https://github.com/FieldRobotEvent/example_ws --recurse-submodules ~/example_ws
rosdep install -y --from-paths ~/example_ws --ignore-src
cd ~/example_ws
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
roslaunch example_robot_brain task_navigation.launch
```

## Build the Docker image of your robot
During the competition, there will be two containers - the simulation container, running rosmaster and gazebo, and the robot container, running your code.
This allows you to have as much freedom as you desire in completing the tasks, without any risk of having mixed dependencies (such as ROS/ROS2) interfering with your solution. To create the robot container, make sure that you installed Docker on our computer. Change the [Dockerfile](Dockerfile) to the requirements of your robot and build the image:

```commandline
docker build . -t robot_workspace
```

After building the image, an image with the name `robot_workspace` should be available in Docker. You can check this by running the `docker images` command. More information how to create your own robot Dockerfile is available in the [competition environment](https://github.com/FieldRobotEvent/competition_environment). 

To test your robot in the competition environment using Docker, clone the [competition environment](https://github.com/FieldRobotEvent/competition_environment) and follow the instructions from there.

To save the docker image as file, run `docker image save robot_workspace | gzip -c - > robot_workspace.tgz`. This can take a few minutes.

## Troubleshooting
| Error | Cause | Solution |
|---|---| --- |
| `Could not open file[(...)/virtual_maize_field/worlds/generated.world]` when launching Gazebo | There is no world file generated. | Generate a world file by e.g. `rosrun virtual_maize_field generate_world.py fre22_task_navigation_mini`. |
| `VMware: vmw_ioctl_command error Invalid argument.` by launching Gazebo | Graphics problem in virtual machine. | Execute `echo "export SVGA_VGPU10=0" >> ~/.profile` in the terminal and reboot your virtual machine. |
| `Error in REST request` when launching Gazebo | Wrong link in Gazebo configuration. | Open `~/.ignition/fuel/config.yaml` and change the line: `url: https://api.ignitionfuel.org` to `url:  https://api.ignitionrobotics.org`. |
| Lidar data on the topic `front/scan` only returns ranges with the value `inf`, even though in simulation the lidar should ‘see’ certain objects within its range. | Graphics problem. | Execute `export LIBGL_ALWAYS_SOFTWARE=1` in the terminal in which you launch gazebo. You have to run this command before starting gazebo. This solves the problem with the lidar, but might have some consequences on the rendering speed of gazebo. |

If you have another error or the provided solution does not work, create a [new issue](https://github.com/FieldRobotEvent/example_ws/issues). Help expanding this list by making a [pull request](https://github.com/FieldRobotEvent/example_ws/pulls).

