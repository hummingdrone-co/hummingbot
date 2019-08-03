# Hummingbot
Hummingdrone Nvidia Jetbot integration

## Getting Started

## Installing

#### Install Gazebo Model

```
cp ./Gazebo/Model/Hummingbot_* ~/.gazebo/models
```

### Install Gazebo Plugin

```
mkdir ~/hummingbot_plugin/
cp ./Gazebo/Plugins/* ~/hummingbot_plugin/ -rf
cd ~/hummingbot_plugin/
mkdir build && cd build
cmake ..
make
```

### Install ROS Packages

```
cp ./ROS/hummingbot/ ~/catkin_ws/src/
cd ~/catkin_ws/ && catkin_make
```
## Check out the Model

* Open terminal
* roslaunch gazebo_ros empty_world.launch
* Insert model from the Insert Tab

## Playing


### Running Hummingbot Package and Topics

* Run
```
roscore
```
* OPEN ANOTHER TERMINAL
```
rosrun hummingbot rover.py
```
* OPEN ANOTHER TERMINAL
```
rosrun hummingbot rover_teleop.py
```
* PUSH (W,A,S,D,X or SPACE) BUTTON in **rover_teleop.py** TERMINAL

### Running Gazebo Model with plugin

*Please make sure that ros master is working before run below commands.
```
cd ~/hummingbot_plugin/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/hummingbot_plugin/build
gazebo --verbose hummingbot_test.world
```

### Test?

* OPEN ANOTHER TERMINAL

```
rostopic echo /left_vel
```
**You should see the velocity of left wheel.**
