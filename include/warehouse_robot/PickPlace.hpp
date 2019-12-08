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
 * @file PickPlace.hpp
 * @brief This file contains the class declaration of PickPlace, which will be
 * using OpenManipulator to pick and place the object
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
#ifndef WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_PICKPLACE_HPP_
#define WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_PICKPLACE_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/SetJointPosition.h>
#include <vector>
#include <utility>
#include "open_manipulator_libs/open_manipulator.h"
/**
 * @brief PickPlace class controls the manipulator on the Turtlebot,
 * enabling it to pick and place the object
 */
class PickPlace {
 public:
  PickPlace();
  ~PickPlace();
  /**
   * @brief Function that executes the pick up operation of manipulator
   * @param ros node handle
   * @return true if succesful
   */
  bool executePick(ros::NodeHandle);

  /**
   * @brief Function that executes the place operation of manipulator
   * @param ros node handle
   * @return true if succesful
   */
  bool executePlace(ros::NodeHandle);

  /**
   * @brief Function that sets the kinematic pose for pick up and calls service
   * @param geometry_msg of type pose, holding the kinematic pose of the end
   * effector
   * @return true if successful
   */
  bool setPose(geometry_msgs::Pose);

  /**
   * @brief Function that controls the gripper
   * @param vector of type double, holding value of joint position of gripper
   * @return true if successful
   */
  bool setGripper(std::vector<double>);

  /**
   * @brief Service to set the kinematics pose of manipulator,
   * service name - /arm/moveit/set_kinematics_pose
   */
  ros::ServiceClient setArmPose;

  /**
   * @brief Service to set the gripper state
   * service name -/om_with_tb3/gripper
   */
  ros::ServiceClient setGripperState;

  /**
   * @brief Pose variable, holding the kinematic pose to pick object
   */
  geometry_msgs::Pose pickPose;

  /**
   * @brief Pose variable, holding the kinematic pose to place object
   */
  geometry_msgs::Pose placePose;

  /**
   * @brief Pose variable, holding the kinematic pose of the home location of
   * the manipulator
   */
  geometry_msgs::Pose homePose;

  /**
   * @brief robot state, 0 if robot is stationary, 1 if it is moving
   */
  bool robotState;

  /**
   * @brief Local node handle
   */
  ros::NodeHandle pnpNode;
};
#endif  // WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_PICKPLACE_HPP_
