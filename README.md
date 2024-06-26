# Human Targeted Delivery Robot

## Installation

### Install ROS Noetic
Follow the instructions to install ROS Noetic for Ubuntu 20.04 at: http://wiki.ros.org/noetic/Installation/Ubuntu. Please install the ros-noetic-desktop-full version

### Install Other ROS Dependencies
```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-depthimage-to-laserscan ros-noetic-gmapping python3-catkin-tools python3-pip ros-noetic-map-server
pip3 install pynput
```

### Create a Catkin Workspace (if none exists)
Follow the instructions at: https://wiki.ros.org/catkin/Tutorials/create_a_workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### Download and Install the HTD Repository
```bash
cd ~/catkin_ws/src
git clone https://gitlab.com/HCRLab/stingray-robotics/cs603_particle_filter.git
cd ~/catkin_ws
catkin_make
```

## Running 

### Generate Map Using GMapping (SLAM)
To generate the map, you must launch the `triton_gmapping.launch` file then move the triton robot around using the teleop file. For more information, you can refer to the turtlebot3_slam page (https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) which is similar but uses a different robot. 

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch cs603_particle_filter triton_gmapping.launch
```

## Performing initial goal room navigation
To activate perform this in simulation, perform the Particle Filter, Move Base/ACML, and Room selection commands. To perform this on a physical robot, don't perform the particle filter commands, since that is for setting up gazebo/rviz.   

## Particle Filter

To run: 
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch cs603_particle_filter particle_filter.launch
```

## Move Base/AMCL
To activate move_base and AMCL nodes, run the following:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch HTD_Robot move_base.launch
```

## Room Selection
To select the goal room to traverse to, run the following:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun HTD_Robot travel_to_room.py
```
