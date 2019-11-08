# ur_robotiq
Collection of packages for using simulated and real universal robots manipulators with Robotiq grippers. This repo uses git submodules, so after cloning the repo you need to update the submodules:


``` bash
git submodule update --init
```
Then, install all of the dependencies using rosdep:
``` bash
rosdep install --from-paths src --ignore-src -r -y
```
Clone the universal_robot repo, specifically the calibration_devel branch to make use of the new calibration parameters that yield higher accuracy motions:
``` bash
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
```

## MoveIt Control Steps:

First start the docker container (docker_image and instructions in uncalibrated_grasping/docker/melodic_ur_robotiq)
Use tmux (or any other teminal splitter) to open 3 terminals and run the following (in this order, waiting for each to complete):
``` bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.90.101 limited:=true info:=true
```
[launch robot in gazebo]
``` bash
roslaunch ur_e_gazebo ur5e_joint_limited.launch
``` 
``` bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch limited:=true info:=true
```
``` bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true

```

If using tmux Ctrl-b followed by " will split the current terminal window vertically, and Ctrl-b followed by up and down arrow keys enable you to move about the different terminal windows.

Next, on the UR5e Teach Pendant, start the external_control program (enabling control from external sources)
TODO: Include image

Now, you can use the standard MoveIt interface to control the arm :)

error:
libGL error: No matching fbConfigs or visuals found

libGL error: failed to load driver: swrast

X Error of failed request:  GLXBadContext

  Major opcode of failed request:  154 (GLX)
  
  Minor opcode of failed request:  6 (X_GLXIsDirect)
  
  Serial number of failed request:  35
  
  Current serial number in output stream:  34
