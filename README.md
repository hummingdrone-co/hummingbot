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

#### Install Joystick
- Install joy package.
```
sudo apt-get install ros-melodic-joy
```
- Configure your joystick to your computer.
```
ls /dev/input
```
- You should see all input devices here. Joysticks are referred to by **jsX**. Our case it is **js1**.

- Test your joystick.
```
sudo jstest /dev/input/js1
```
* Move your joystick and see the data changes.

- Make your joystick accesible for ROS node with command below.
```
sudo chmod a+rw /dev/input/js1
```

## Check out the Model

* Open terminal
```
roslaunch gazebo_ros empty_world.launch 
```
* Insert model from the Insert Tab

## PLAYING
- > **WARNING**: If your joystick reference is different than 'js1':
    - Please find the command below in **hummingbot.launch** file and change the value part with yours.
        ```
        <param name="dev" type="string" value="/dev/input/js1" /> 
        ```
- Export your build directory
```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/hummingbot_plugin/build
```
* **HINT : In order to run your plugins you should run the command above in every terminal session. But there is an easy way as always! If you do not want to write it everytime then you can add it into your .bashrc file with the commands below.**

    ``` 
    echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/hummingbot_plugin/build" >> ~/.bashrc
    source ~/.bashrc
    ```
- **Finally, run and control your hummingbot!**
```
roslaunch hummingbot hummingbot.launch
```



### Test?

```
rostopic echo /gazebo_hummingbot_client/left_vel
```
**You should see the velocity of left wheel.**
