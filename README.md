# Gazebo Tutorial
Implement a simple wall avoider algorithm much like a Roomba robot vacuum cleaner. 

## Author
- Manu Madhu Pillai (117817928)

## Dependencies and environment setup

### Docker Setup
Using a docker image of ROS2 Humble.

```bash
#Pull the docker image
docker pull osrf/ros:humble-desktop-full
# Run ROS 2 container
rocker osrf/ros:humble-desktop-full bash

# inside ROS 2 container, install terminator
apt update;
apt -y install terminator
apt -y install ros-humble-gazebo-ros-pkgs 
apt -y install ros-humble-turtlebot3*
apt clean all

#Use another terminal to save the docker image
CONTAINER_ID=$(docker ps --format {{.ID}})
echo $CONTAINER_ID

# Save the Docker image snapshot 
docker commit $CONTAINER_ID my-docker2-humble
```
The environment is setup and can be used with the following command:
```bash
rocker --x11 --user --home --nvidia  --privileged my-docker2-humble terminator
```

### Build respository
Create a workspace and clone the repository to the src folder
```bash
mkdir -p ~/rosws/src
cd ~/rosws/src
git clone https://github.com/lilnpuma/gazebo_tutorial.git
cd ..
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1 
export ROS_LOCALHOST_ONLY=1
export TURTLEBOT3_MODEL=waffle
colcon build
```

## simple_turtle
It contains a 'cmd_vel' publisher that 'scan' topic to avoid obstacles ahead of it.
```bash
cd ~/ws/
. install/setup.bash
ros2 run simple_turtle controller
```
 To launch both the controller and the gazebo environment in a custom world

 ```bash
 ros2 launch simple_turtle simple_turtle.launch.py
 ```
 To enable recording
 ```bash
 ros2 launch simple_turtle simple_turtle.launch.py record_enabled:='True'
 ```
 To replay a rosbag along with the simulation to replay the simulation(read and publish only the /cmd_vel topic that was recorded)
```bash
ros2 launch simple_turtle simple_turtle.launch.py replay_only:='True' bag_file:='./src/simple_turtle/results/recorder1670292035.7707734.bag'
 ```
 To inspect the bag file:
 ```bash
 ros2 bag info ./src/simple_turtle/results/recorder1670292035.7707734.bag'
 ```
 To play the bag file:
 ```bash
 ros2 bag play ./src/simple_turtle/results/recorder1670292035.7707734.bag'
 ```

 ## Issues Found
 - If GAZEBO_MODEL_PATH and TURTLEBOT3_MODEL paths are not set, the launch file wont run.
 - Check if all dependencies are met for proper operation of the package.

 ## Code Checks

 ### cppcheck
 Run this command to get the output.
 ```bash
 cd ~/rosws/src
 cppcheck --enable=all --std=c++17 src/*.cpp include/simple_turtle/*.hpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression
```

### cpplint
Run this command to get the output.
 ```bash
 cd ~/rosws/src
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp include/simple_turtle/*.hpp 
```