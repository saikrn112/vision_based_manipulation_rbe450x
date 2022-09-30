# Gazebo Installation Tutorials
**RBE 450X Fall A-22**

*Made by Prof. Berk Calli and Yash Patil*

In this tutorial we will install Gazebo 11 and all its supporting files that will aid us in robot simulation.

## Gazebo
Use the following commands to install Gazebo and its supplementry files
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```  
Once this is done, update using and make sure it runs without any errors 
```
sudo apt-get update
```
now install Gazebo using 
```
sudo apt-get install gazebo libgazebo-dev
```
You can check your installation by running this in a new terminal 
```
gazebo
```

## Gazebo ROS 2 packages
To use Gazebo with ros2, there are certain packages that need to be installed
```
sudo apt install ros-humble-gazebo-ros-pkgs
```
Now your system is ready to simulate any robot.

## Spawn rrbot in Gazebo
These are optional steps just to make sure everything is alright.

Please find the zipped package that has one of many ways to spawn an URDF in Gazebo using ROS2.

Extract the package to your workspace/src.

Before doing `colcon build` we have to install some dependencies for the package to function correctly.
Run the following commands in terminal
```
sudo apt install python3-pip
sudo pip3 install transforms3d
```
Now navigate to your workspace and run 
```
colcon build --symlink-install
```
You can verify if the build was successful by navigating to install directory in workspace and finding rrbot_description_ros2 package there.
Now simply open a new terminal and navigate to the workspace, source it just in case.
```
. install/setup.bash
```
Launch Gazebo and Spawn the RRbot
```
ros2 launch rrbot_description_ros2 simple_launch.py
```
This will result in a new Gazebo window and rrbot will be spawned in it.

*Note: This can also result in your system getting hanged for a few minutes, this is just a first time thing and please don't kill the node if this happens, average hang-time is around 2-3 mins, if it goes above 10 mins kill the launch file.*
### You can refer to this package and understand how the robots can be spawned in Gazebo. 
