# ur_robotiq
Collection of packages for using simulated and real universal robots manipulators with Robotiq grippers. This repo uses git submodules, so after cloning the repo you need to update the submodules:


``` bash
git submodule update --init
```
``` bash
rosdep install --from-paths src --ignore-src -r -y
```

## MoveIt Control Steps:

First start the docker container (docker_image and instructions in uncalibrated_grasping/docker/melodic_ur_robotiq)

Next open 3 terminals and run the following (in this order, waiting for each to complete):
``` bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.90.101 limited:=true info:=true
```
``` bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch limited:=true info:=true
```
``` bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true

```

Next, on the UR5e Teach Pendant, start the external_control program (enabling control from external sources)
TODO: Include image

Now, you can use the standard MoveIt interface to control the arm :)
