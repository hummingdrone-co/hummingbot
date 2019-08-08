# Hummingbot
Hummingdrone Nvidia Jetbot integration

## Getting Started

## Installing

#### Install Gazebo Model

```
cp ./Gazebo/Model/Hummingbot_* ~/.gazebo/models
```

#### Install Gazebo Plugin

```
mkdir ~/catkin_ws/src/hummingbot_plugin/
cp ./Gazebo/Plugins/* ~/catkin_ws/src/hummingbot_plugin/ -rf
cd ~/catkin_ws/src/hummingbot_plugin/
mkdir build && cd build
cmake ..
make
```

#### Install ROS Packages

```
cp ./ROS/hummingbot/ ~/catkin_ws/src/
cd ~/catkin_ws/ && catkin_make
```
## Check out the Model

* Open terminal
```
roslaunch gazebo_ros empty_world.launch 
```
* Insert model from the Insert Tab

## PLAYING

#### Running Gazebo
```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/hummingbot_plugin/build
```
* **HINT : In order to run your plugins you should run the command above in every terminal session. But there is an easy way as always! If you do not want to write it everytime then you can add it into your .bashrc file with the commands below.**

    ``` 
    echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/hummingbot_plugin/build" >> ~/.bashrc
    source ~/.bashrc
    ```

```
roslaunch hummingbot hummingbot.launch
```

#### Running Teleoperation
```
rosrun hummingbot rover_teleop.py
```

Control your hummingbot from this terminal!


### Test?

```
rostopic echo /gazebo_hummingbot_client/left_vel
```
**You should see the velocity of left wheel.**
