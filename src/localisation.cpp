/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 Suyash Yeotikar, Yashaarth Todi, Gautam Balachandran
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  @file localisation.cpp
 *  @date Nov 28, 2019
 *  @author Suyash Yeotikar (test-driver), Yashaarth Todi (development - driver)
 *  @brief main file
 *  Copyright 2019 Suyash Yeotikar, Yashaarth Todi, Gautam Balachandran  [legal/copyright]
 *  @mainpage project page
 *  This project is developed for Warehouse Management wherein a Turtlebot3
 *  Waffle Pi is employed to transport packages from one location to another.
 *  The robot detects the location of the package from the Aruco marker and
 *  will drop the package to a user defined location.Turtlebot3 uses
 *  OpenManipulator for its pick and place operation,
 *  and would be using A-star algorithm to plan its path.
 */
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include  <cstdlib>
#include "localisation.hpp"

void Localisation::EntropyCallback(const std_msgs::Float64::ConstPtr &msg) {
  entropy = msg->data;
}

bool Localisation::SetEntropyThreshold(double thresholdValue) {
  if (thresholdValue > 0) {
    entropyThreshold = thresholdValue;
    return true;
  } else {
    ROS_WARN_STREAM(" Threshold value must be positive, using default value");
    // Setting default value of entropy threshold
    entropyThreshold = 5;
    return false;
  }
}

void Localisation::GetRobotCoordinate(tf::StampedTransform mapToRobot) {
  try {
    mapToBaseTfListen.lookupTransform("/map", "/om_with_tb3/base_footprint",
                                      ros::Time(0), mapToRobot);
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM("Exception : " << ex.what());
    ros::Duration(1.0).sleep();
  }
  // Getting pose of robot
  localisationPose.position.x = mapToRobot.getOrigin().x();
  localisationPose.position.y = mapToRobot.getOrigin().y();
  localisationPose.position.z = mapToRobot.getOrigin().z();
  localisationPose.orientation.x = mapToRobot.getRotation().x();
  localisationPose.orientation.y = mapToRobot.getRotation().y();
  localisationPose.orientation.z = mapToRobot.getRotation().z();
  localisationPose.orientation.w = mapToRobot.getRotation().w();

  // Assuming we have receieved correct pose
  bool poseCheck = true;

  tf::Quaternion poseQuat(localisationPose.orientation.x,
                          localisationPose.orientation.y,
                          localisationPose.orientation.z,
                          localisationPose.orientation.w);
  tf::Matrix3x3 euler(poseQuat);
  double roll, pitch, yaw;
  // Converting quarternion to euler angles
  euler.getRPY(roll, pitch, yaw);
  // Checking if the robot has toppled
  if (abs(roll) || abs(pitch) >= 0.1745) {
    ROS_WARN_STREAM(
        "Robot localisation unsuccessful- pose is not correct, seems to" <<
        " have fallen");
    poseCheck = false;
  }
  // Checking if the received position is out of the map
  if (localisationPose.position.x > 38.95 || localisationPose.position.x < 0
      || localisationPose.position.y > 20.1 || localisationPose.position.y < 0
      || localisationPose.position.z > 0.15
      || localisationPose.position.z < -0.01) {
    ROS_WARN_STREAM(
        "Robot  localisation unsuccessful- pose is not correct,seems out of" <<
        ", bounds of map");
    poseCheck = false;
  }
  // Checking if the reliability of the pose recieved is lower than threshold
  if (entropy > entropyThreshold) {
    ROS_WARN_STREAM(
        "Robot localisation unsuccessful- pose is not correct, high " <<
        "uncertainity of Robot pose");
    poseCheck = false;
  }
  if (poseCheck == false) {
    // Discarding recieved pose in case above conditions are met
    localisationPose.position.x = 0;
    localisationPose.position.y = 0;
    localisationPose.position.z = 0;
    localisationPose.orientation.x = 0;
    localisationPose.orientation.y = 0;
    localisationPose.orientation.z = 0;
    localisationPose.orientation.w = 0;
    return;
  }
  ROS_INFO_STREAM("Localisation Succesful");
}

void Localisation::PublishMapPose() {
  geometry_msgs::Pose localMapPose;
  // Converting actual pose in map to pose percieved in pixels
  localMapPose.position.x = localisationPose.position.x / 0.05;
  localMapPose.position.y = localisationPose.position.y / 0.05;
  localMapPose.position.z = localisationPose.position.z / 0.05;
  localMapPose.orientation = localisationPose.orientation;
  // Publishing pixel pose to topic mapPose
  mapPosePublisher.publish(localMapPose);
}

void Localisation::PublishRawPose() {
  // Publishing robot pose to topic rawPose
  rawPosePublisher.publish(localisationPose);
}

void Localisation::initSubscribers(ros::NodeHandle n) {
  // Setting local node handle to master node handle
  // Subscribing to entropy topic
  localizationNode = n;
  entropySubscriber =
      localizationNode.subscribe < std_msgs::Float64
          > ("/turtlebot3_slam_gmapping/entropy", 1000,
          &Localisation::EntropyCallback, this);

  // Defining publisher for mapPose topic
  mapPosePublisher = localizationNode.advertise < geometry_msgs::Pose
      > ("/mapPose", 1000);

  // Defining publisher for rawPose topic
  rawPosePublisher = localizationNode.advertise < geometry_msgs::Pose
      > ("/rawPose", 1000);
}

void Localisation::ExecuteLocalisation() {
  tf::StampedTransform mapToRobot;
  GetRobotCoordinate(mapToRobot);
  PublishMapPose();
  PublishRawPose();
}
