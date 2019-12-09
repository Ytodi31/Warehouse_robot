/**
 *BSD 3-Clause License
 *
 *Copyright (c) 2019, Yashaarth Todi
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file PickPlace.cpp
 * @brief This file contains the function defintions of class PickPlace
 *
 * This project contains the execution to navigate Turtlebot3 in a warehouse
 * environment using A star path planning, picks up a package and drops it in
 * user defined postitions. Turtebot3 uses OpenManipulator to perform thus
 * pick-place task.
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Suyash Yeotikar
 * @author Gautam Balachandran
 * @author Yashaarth Todi
 *
 * @date 11-28-2019
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/SetJointPosition.h>
#include <vector>
#include <utility>
#include "PickPlace.hpp"

PickPlace::PickPlace() {
  // Setting home position of manipulator
  homePose.position.x = 0.198448840528;
  homePose.position.y = 6.60363761286e-05;
  homePose.position.z = 0.301622770805;
  homePose.orientation.x = -5.11700663643e-07;
  homePose.orientation.y = 0.00431527188029;
  homePose.orientation.z = 0.000118577904216;
  homePose.orientation.w = 0.99999068214;
}

bool PickPlace::setPose(geometry_msgs::Pose armPose) {
  // Creating service client to set manipulator pose to pick object
  setArmPose = pnpNode.serviceClient < open_manipulator_msgs::SetKinematicsPose
      > ("/arm/moveit/set_kinematics_pose");
  // Defining reuqest and response for service
  open_manipulator_msgs::SetKinematicsPose::Request initPose;
  open_manipulator_msgs::SetKinematicsPose::Response setPoseResp;
  // Defining message to be sent over service to set kinematic pose
  initPose.planning_group = "";
  initPose.end_effector_name = "";
  initPose.kinematics_pose.pose.position.x = armPose.position.x;
  initPose.kinematics_pose.pose.position.y = armPose.position.y;
  initPose.kinematics_pose.pose.position.z = armPose.position.z;
  initPose.kinematics_pose.pose.orientation.x = armPose.orientation.x;
  initPose.kinematics_pose.pose.orientation.y = armPose.orientation.y;
  initPose.kinematics_pose.pose.orientation.z = armPose.orientation.z;
  initPose.kinematics_pose.pose.orientation.w = armPose.orientation.w;
  initPose.kinematics_pose.max_accelerations_scaling_factor = 1;
  initPose.kinematics_pose.max_velocity_scaling_factor = 1;
  initPose.kinematics_pose.tolerance = 0.01;
  initPose.path_time = 0.0;
  // Calling and returning response from service
  return setArmPose.call(initPose, setPoseResp);
}

bool PickPlace::setGripper(std::vector<double> state) {
  // Creating service client to set gripper position
  setGripperState = pnpNode.serviceClient
      < open_manipulator_msgs::SetJointPosition > ("/om_with_tb3/gripper");
  // Defining request and response for service
  open_manipulator_msgs::SetJointPosition::Request initialState;
  open_manipulator_msgs::SetJointPosition::Response finalState;
  // Defining message to be sent over service to set gripper position
  initialState.planning_group = "";
  initialState.joint_position.position = state;
  initialState.joint_position.max_accelerations_scaling_factor = 1;
  initialState.joint_position.max_velocity_scaling_factor = 1;
  initialState.path_time = 0;
  // Calling and returning response from service
  return setGripperState.call(initialState, finalState);
}

bool PickPlace::executePick(ros::NodeHandle m) {
  // Setting main node handle to local handle
  pnpNode = m;
  // Sending command to open gripperState
  std::vector<double> open { 0.01 };
  bool gripperState = setGripper(open);
  ros::Duration(1).sleep();
  bool pickSet = setPose(pickPose);
  ROS_INFO_STREAM("SETTING POSE FOR PICKING!");
  ros::Duration(1).sleep();
  // Closing gripper if pose was succesfully set
  if (pickSet == true) {
    std::vector<double> close { -0.01 };
    gripperState = setGripper(close);
    ROS_INFO_STREAM("Closing gripper");
  }
  // Setting manipulator to home position
  bool goHome = setPose(homePose);
  return goHome;
}

bool PickPlace::executePlace(ros::NodeHandle m) {
  // Setting main node handle to local handle
  pnpNode = m;
  // Creating service client to set manipulator pose to place object
  setArmPose = pnpNode.serviceClient < open_manipulator_msgs::SetKinematicsPose
      > ("/arm/moveit/set_kinematics_pose");
  bool placeSet = setPose(placePose);
  ROS_INFO_STREAM("SETTING POSE FOR PLACING!");
  ros::Duration(1).sleep();
  // Creating service client to set gripper position
  setGripperState = pnpNode.serviceClient
      < open_manipulator_msgs::SetJointPosition > ("/om_with_tb3/gripper");
  if (placeSet == true) {
    std::vector<double> state { 0.01 };
    bool gripperState = setGripper(state);
    ROS_INFO_STREAM("Opening gripper");
  }
  // Setting manipulator to home position
  bool goHome = setPose(homePose);
  return goHome;
}

PickPlace::~PickPlace() {
}
