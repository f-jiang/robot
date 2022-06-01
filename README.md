# Mobile Robot

[BOM](https://docs.google.com/spreadsheets/d/1hAyx_H2yaLHObjr3c2HcGa4GbXQhFF7O3Fq-fULQeV0/edit?usp=sharing)
[Google Drive](https://drive.google.com/drive/folders/1Uw0iWY_K3mZgV3rem3GLMkO6ajqQ58nl)

<img src="https://raw.githubusercontent.com/f-jiang/robot/master/images/robot.jpg" width="600">

### Contributors:

- Feilan Jiang
- Matt Strong
- Tejit Pabari
- Satish Upadhyaya

## Requirements

- ROS Melodic

## Setup

```bash
# create a catkin workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/f-jiang/robot
cd ..

# automatically install the packages
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
catkin build 
# or catkin_make

# it is recommended to add the below line to your ~/.bashrc
source devel/setup.bash
```

## Running the Simulation 

```bash
# run the simulation
roslaunch world_name:=worlds/rooms.world use_gazebo:=true robot_base base.launch

# run teleop (currently requires a joystick, such as a Xbox Controller)
roslaunch use_gazebo:=true robot_control teleop.launch
```

This is how the simulation should look:

<img src="https://raw.githubusercontent.com/f-jiang/robot/master/images/robot_in_gazebo.png" width="600">

