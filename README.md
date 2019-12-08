# Autonomous Warehouse Management Robot (AuWaMaR)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/Ytodi31/warehouse_robot.svg?branch=iteration3)](https://travis-ci.org/Ytodi31/warehouse_robot)
[![Coverage Status](https://coveralls.io/repos/github/Ytodi31/warehouse_robot/badge.svg?branch=iteration3)](https://coveralls.io/github/Ytodi31/warehouse_robot?branch=iteration3)

## Overview
This project is developed for Warehouse Management wherein a Turtlebot3 Waffle Pi
is employed to transport packages from one location to another. The pick-up and drop
locations are specified by the user.Turtlebot3 uses OpenManipulator for its
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
- Gmapping
- Turtlebot3 with OpenManipulator
- CV_Bridge

Ensure the above packages are installed before running the project.

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


## License Overview
BSD 3-Clause License
Copyright (c) 2019, Yashaarth Todi
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
