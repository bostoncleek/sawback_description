### Author: Boston Cleek
### Date: 5/25/20

# Overview
This package contains the files required for simulating the Sawback in gazebo and in rviz. The Sawback is comprised of both a Ridgeback mobile base and a Sawyer manipulator. The following sensors are includes in the model: Velodyne VLP16 Lidar, 2 Hokoyu Lidars, and Bumblebee2 Stereo Camera. The Sawback is the ruler of the Planet Omicron Persei 8.


# How to run:
The easiest way to get started is to download the rosinstall file or you will need the required packages bellow. In the src/ folder of your workspace run the following:

You may also need to install the `velodyne-gazebo-plugins` package and the `effort_controllers` package.


`mkdir -p catkin_ws/src` <br/>
`cd ~/catkin_ws/src` <br/>
`wstool init .` <br/>
`wstool merge -t . https://raw.githubusercontent.com/bostoncleek/sawback_description/master/sawback.rosinstall` <br/>
`wstool update -t .` <br/>
`cd ~/catkin_ws` <br/>
`catkin_make`


To vizualize the sawback in rviz: <br/>
`roslaunch sawback_description view_sawback.launch`

To simulate the sawback in gazebo: <br/>
`roslaunch sawback_description sawback_world.launch`

To do both: <br/>
`roslaunch sawback_description sawback_world.launch launch_rviz:=true`



# Required Packages
1) velodyne-gazebo-plugins <br/>
2) effort_controllers <br/>
3) ridgeback_control <br/>
4) ridgeback_description <br/>
5) sawyer_description <br/>
6) sawyer_gazebo <br/>
7) sawyer_sim_controllers


# Known Issues
There is a race condition between ros_control and gazebo when spawning a urdf with initial joint angles using the `-J` arg in the `spawn_model` node. The simplest solution is that gazabo must be paused during startup and the `-unpause` arg must be passed to the `spawn_model` node. See [spawn_model -J initial joint positions not working #93](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/93) for more information. There are many suggestions in this thread. Some users suggest the order in which ros_control and gazebo services are called can prevent the race condition. Based on the testing results the order in which these services are called does not reliably prevent the issue. Another alternative is to write a world plugin that dynamically creates a joint between the two robots at run time. The world plugin does exist in the git commits for this package that can do this. See SHA1 ID: 988d428 ("world plugin to dynamically create a joint between the robots at run time", 2020-05-25)


# The Sawback
<p align="center">
  <img src="media/sawback_gazebo.png" width="400" height="350"/>
  <img src="media/sawback_rviz.png" width="400" height="350"/>
</p>
