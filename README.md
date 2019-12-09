# Autonomous Warehouse Management Robot (AuWaMaR)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/Ytodi31/warehouse_robot.svg?branch=master)](https://travis-ci.org/Ytodi31/warehouse_robot)
[![Coverage Status](https://coveralls.io/repos/github/Ytodi31/warehouse_robot/badge.svg?branch=master)](https://coveralls.io/github/Ytodi31/warehouse_robot?branch=master)

## Overview
This project is developed for Warehouse Management wherein a Turtlebot3 Waffle Pi
is employed to transport packages from one location to another. The robot detects
the location of the package from the Aruco marker and will drop the package
to a user defined location.Turtlebot3 uses OpenManipulator for its
pick and place operation, and would be using A-star algorithm to plan its path.

__Technical Presentation__\
A brief Technical Presentation of the project can be found
[here](https://docs.google.com/presentation/d/14QqkwVFgyVMpcSMVNDBS_6l7sLMyVfS4-13v47q9Lyo/edit?usp=sharing)

## Development Team
- Yashaarth Todi is Robotics Graduate student at University of Maryland, College Park.
- Suyash Yeotikar is Robotics Graduate student at University of Maryland, College Park.
- Gautam Balachandran is Robotics Graduate student at University of Maryland, College Park.

## AIP
The development team has followed AIP process. The link to AIP sheet can be found
[here](https://docs.google.com/spreadsheets/d/1KsJT0aIaXuEaXX1XelDLg1EV10q6P--KvlY-5qnYGCg/edit#gid=0). \
The AIP planning and review sheet used by the team can be be found [here](https://docs.google.com/document/d/1O4B-fgY8ZTwz_BBVik3APsgOGGVCKxhNlC9fsOs6aUY/edit).


---
## Dependencies
- The project uses Ubuntu 16.04
- The project uses Kinetic version of ROS. To install, follow the [link]( http://wiki.ros.org/kinetic/Installation/Ubuntu)
- The project uses catkin build system, To install, follow the [link](http://wiki.ros.org/catkin)
- The project uses Google Test framework for Unit Testing.

The following ROS packages are used in the project:
- [gmapping](http://wiki.ros.org/gmapping)
- [om_with_tb3](http://wiki.ros.org/open_manipulator_with_tb3_waffle_pi_moveit) (OpenManipulator with Turtlebot3)
- [cv_bridge](http://wiki.ros.org/cv_bridge)
- [open_manipulator_msgs]([https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator_libs]([http://wiki.ros.org/open_manipulator)
- [aruco](http://wiki.ros.org/aruco)
- [roslib](http://wiki.ros.org/roslib)\
Ensure the above packages are installed before running the project.

---
## Building and Running the project

### Build
1. Create a catkin workspace \
`mkdir -p ~/catkin_ws/src` (Skip this step if you have an exisitng catkin worksapce)\
`cd ~/catkin_ws/` \
`catkin_make`
2. Source the new setup files \
`source ./devel/setup.bash`
3. Clone the repository\
`cd src/` \
`git clone https://github.com/Ytodi31/warehouse_robot.git`
4. Build the project \
`cd ..` \
`catkin_make`
5. To install custom service\
`catkin_make install`
---

### Run
The program can be started with following the steps below:
- In terminal 1, the model Turtlebot3 is exported and the launch file brings up
the robot in the environment.
- In terminal 2, the commands start the software module to run the robot.
- In terminal 3, the user provides the goal point to the robot using service caller.

1. Terminal 1\
`source ./devel/setup.bash` \
`export  TURTLEBOT3_MODEL=${TB3_MODEL}`\
`export GAZEBO_MODEL_PATH=/home/ytodi31/warehouse_robot/src/warehouse_robot/data/models`\
`roslaunch warehouse_robot final.launch`\

2. Terminal 2\
`source ./devel/setup.bash`\
`rosrun warehouse_robot warehouseRobot`

3. Terminal 3\
`source ./devel/setup.bash`\
`rosrun rqt_service-caller rqt_service_caller`\
Navigate service caller to /user_input.
Here the goal point will have to be provided by the user which is of the
[geometry_msg/Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html).

By default, the recording is off. The rosbag will record all topic except
topics related from camera. It can be turned on by passing an argument in
Terminal when doing roslaunch. A sample is shown below :
`roslaunch warehouse_robot final.launch recordBag:=true`

---
## Demo

<p align="center">
  <img width="250" height="250" src="https://github.com/Ytodi31/warehouse_robot/blob/iteration3/data/images/image1.png">

  <img width="250" height="250" src="https://github.com/Ytodi31/warehouse_robot/blob/iteration3/data/images/image2.png">
</p>
Left image shows the robot in the Gazebo and Rviz environment.
Right image shows the robot using Gmapping to get robot localisation in Rviz environment.


A video of the robot at work can be found [here](https://drive.google.com/file/d/1FUNPBxU_4WUri4vttHXVxIPHSgaNAUZ-/view?usp=sharing).


### Expectation

On launching and running the project, you should see the robot move towards
the box having aruco marker and then move towards the goal pose mentioned by the user.
The robot will move from its start point only if the Astar algorithm was able to generate the path while avoiding obstacles.
The robot will move after reaching the pickup point only if it picks up the box succesfully.

---

## Unit Testing
The module makes use of Level 1 Testing - Unit testing with Gtest and Level 2
Testing - Integration testing with rostest.

### Building the test
To build the tests, run the following command:\
`cd ~/catkin_ws/`\
`source ./devel/setup.bash` \
`catkin_make test`

### Running the test
To run the tests and view the results of the test\
`catkin_make run_tests`

Alternatively,
- The tests can be runned using a launch file\
`roslaunch warehouse_robot allTestWarehouseRobot.test `

- The tests can be runned as a node\
Terminal 1:\
`roscore`\
Terminal 2:\
`rosrun warehouse_robot allTestWarehouseRobot`

---


## License disclaimer
The project is released under BSD 3-Clause license and the preamble can be found
below:

BSD 3-Clause License
Copyright (c) 2019, Yashaarth Todi
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
