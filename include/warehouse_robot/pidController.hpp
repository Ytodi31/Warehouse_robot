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
 * @file pidController.hpp
 * @brief This file provides the header file for the Controller module
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
 * @date 11-27-2019
 */

#ifndef INCLUDE_WAREHOUSE_ROBOT_PIDCONTROLLER_HPP_
#define INCLUDE_WAREHOUSE_ROBOT_PIDCONTROLLER_HPP_

#include <math.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

/*
 * pidController.hpp
 *
 *  Created on: Nov 26, 2019
 *  Author: Gautam Balachandran
 */

class PidController {
 private:
  // ROS Node handle object for controller
  ros::NodeHandle controllerNode;
  // Subscriber Object for Pose
  ros::Subscriber poseSub;
  tf::Pose pose;
  double linearVel;
  std::vector<double> kP;
  std::vector<double> kI;
  std::vector<double> kD;
  double lastLinearError = 0;
  double sumLinearError = 0;
  double linearVelThreshold = 0.8;
  double angularVelThreshold = 0.1;

 public:
  double angularVel;
  double sumAngularError = 0;
  double lastAngularError = 0;
  // ROS publisher object for velocity publishing
  ros::Publisher velocityPub;
  /**
   * @brief Setter method for the Ros Node
   * @param New Node to be set
   * @return none
   */
  void setControllerNode(ros::NodeHandle n);
  /**
   * @brief Getter method for the velocity publisher
   * @param none
   * @return The current velocity publisher
   */
  void setVelocityPub();

  /**
   * @brief Setter method for the pose subscriber
   * @param New pose subscriber to be set
   * @return none
   */
  void setPoseSub();
  /**
   * @brief Getter method for the pose
   * @param none
   * @return The current pose of the turtlebot
   */
  tf::Pose getPose();
  /**
   * @brief Getter method for the linear velocity
   * @param none
   * @return The current linear velocity of the turtlebot
   */
  double getLinearVel();

  /**
   * @brief Getter method for the angular velocity
   * @param none
   * @return The current angular velocity of the turtlebot
   */
  double getAngularVel();
  /**
   * @brief Getter method for the Propotional gain
   * @param none
   * @return The current propotional gain of the controller
   */
  std::vector<double> getKP();
  /**
   * @brief Setter method for the propotional gain
   * @param New propotional gain to be set
   * @return none
   */
  void setKP(double kP_linear, double kP_angular);
  /**
   * @brief Getter method for the derivative gain
   * @param none
   * @return The current derivative gain of the controller
   */
  std::vector<double> getKD();
  /**
   * @brief Setter method for the derivative gain
   * @param New derivative gain to be set
   * @return none
   */
  void setKD(double kD_linear, double kD_angular);
  /**
   * @brief Getter method for the integral gain
   * @param none
   * @return The current integral gain of the controller
   */
  std::vector<double> getKI();
  /**
   * @brief Setter method for the integral gain
   * @param New integral gain to be set
   * @return none
   */
  void setKI(double kI_linear, double kI_angular);
  /**
   * @brief Callback function to get the pose data from the Turtlebot
   * @param Turtlebot Pose
   * @return none
   */
  void distCallBack(const geometry_msgs::PoseStamped::ConstPtr &distMsg);
  /**
   * @brief Euclidean distance calculator
   * @param Current pose and desired pose
   * @return Calculated distance
   */
  double euclideanDist(tf::Pose currentPos, tf::Pose desiredPos);
  /**
   * @brief Method to calculate the linear and angular velocity for the robot
   * @param Current pose and desired pose
   * @return Calculated linear and angular velocity
   */
  void calcVel(tf::Pose currentPos, tf::Pose desiredPos);
};

#endif  // INCLUDE_WAREHOUSE_ROBOT_PIDCONTROLLER_HPP_
