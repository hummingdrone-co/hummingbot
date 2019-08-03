# Hummingbot
Hummingdrone Nvidia Jetbot integration

## Getting Started

## Installing

#### Gazebo Models

```
cp ./Gazebo/Model/Hummingbot_* ~/.gazebo/models
```

## Playing with Model

* Open terminal
* roslaunch gazebo_ros empty_world.launch
* Insert model from the Insert Tab

#### ROS Packages

```
cp ./ROS/hummingbot/ ~/catkin_ws/src/
cd ~/catkin_ws/ && catkin_make
```

## Running Hummingbot Package and Topics
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
* OPEN ANOTHER TERMINAL

```
rostopic echo /left_vel
```
**You should see the velocity of left wheel.**
