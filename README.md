# Hummingbot
Hummingdrone Nvidia Jetbot integration

## Getting Started

## Installing

#### Install Gazebo Model

```shell
cp -r ./Gazebo/Model/hummingbot ~/.gazebo/models
```

#### Install Gazebo Plugins

```shell
mkdir ~/catkin_ws/src/hummingbot_plugin/
cp -r ./Gazebo/Plugins/* ~/catkin_ws/src/hummingbot_plugin/ -rf
```

#### Install ROS Packages

```shell
cp -r ./ROS/hummingbot/ ~/catkin_ws/src/
```

#### Build Your Packages and Plugins

```shell
cd ~/catkin_ws/ && catkin_make
```

#### Install Joystick

- Install joy package.
```shell
sudo apt-get install ros-melodic-joy
```
- Configure your joystick to your computer.
```shell
ls /dev/input
```
- You should see all input devices here. Joysticks are referred to by **jsX**. Our case it is **js1**.

- Test your joystick with command below.
```shell
sudo jstest /dev/input/js1
```
* Move your joystick and see the data changes.

- Make your joystick accessible for ROS node with command below.
```shell
sudo chmod a+rw /dev/input/js1
```

## Check out the Model

* Open terminal
```shell
roslaunch gazebo_ros empty_world.launch 
```
* Insert model from the Insert Tab

## Playing
- > **WARNING**: If your joystick reference is different than 'js1':
    - Please find the command below in **simulation.launch** file and change the value with yours.
        ```xml
        <param name="dev" type="string" value="/dev/input/js1" /> 
        ```
- Export your build directory
```shell
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/hummingbot_plugin/build
```
* **HINT : In order to run your plugins you should run the command above in every terminal session. But there is an easy way as always! If you do not want to write it every time then you can add it into your .bashrc file with the commands below.**

    ``` shell
    echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/hummingbot_plugin/build" >> ~/.bashrc
    source ~/.bashrc
    ```
- **Finally, run and control your hummingbot!**
```shell
roslaunch hummingbot simulation.launch
```

## Testing

- **Testing Stop Service**

```shell
rostest hummingbot hummingbot.test
```

You should see the output below. That means all tests passed and succeed.

```
SUMMARY
 * RESULT: SUCCESS
 * TESTS: 3
 * ERRORS: 0
 * FAILURES: 0
```

In this case there are 3 test types for the velocities of any wheel.

- greater or equal
- less or equal
- **equal to zero (when stop service is called it should be zero)**

**Important:** No need to check the type of variable. ROS does not allow to any topic that publishes a variable different than its type. 

Now joy message includes stop button too. If you remove stop button from the message, test_stop_service method should be failed. **That also shows our stop service is working!**

Lets remove it and try again!

```shell
cd ROS/hummingbot/scripts/test/
```

Find here **publisher.py** file and **BUTTON_MSG** constant inside of the file.

Stop button of joystick is represented as **BUTTON_MSG[3]**.

- If you make that value zero, means no call for stop service.
- If you make that value one, means there is a call for stop service.

So change it as zero, and call test file again. You should see the output below.

```
SUMMARY
 * RESULT: FAIL
 * TESTS: 3
 * ERRORS: 0
 * FAILURES: 1
```

## Remote ROS Connection

Here explains how to start and control ROS system using multiple machines. ***ROS_MASTER_URI*** let you  configure multiple machines to use a **single** ROS master.

Lets think;

- First machine's IP address : 192.168.1.1

- Second machine's IP address: 192.168.1.2

First runs motor and control. Second runs joystick.

- Export IP addresses **for both of them**.

```shell
export ROS_IP="Machine's own IP ADDRESS"
```

- For **SECOND** machine:

```shell
export ROS_MASTER_URI=http://192.168.1.1:11311 
# exports the ros master's port of first machine
```

- For **FIRST** machine:

```shell
roslaunch hummingbot robot.launch
```

- For **SECOND** machine:

```shell
roslaunch hummingbot remote.launch
```

**Now you should control your hummingbot by using remote machine!**