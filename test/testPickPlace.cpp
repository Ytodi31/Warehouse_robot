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
 * @file testPickPlace.cpp
 * @brief This file tests the member functions of class PickPlace
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
 * @date 11-30-2019
 */
 #include "ros/ros.h"
 #include "gtest/gtest.h"
 #include <open_manipulator_msgs/GetKinematicsPose.h>
 #include "PickPlace.hpp"

 /**
  * @brief Test to check the member function setPose of class
  * PickPlace
  */
TEST(TestPickPlace, testSetPose) {
  geometry_msgs::Pose testPose;
  ros::NodeHandle t;
  testPose.position.x = 0.198448840528;
  testPose.position.y = 6.60363761286e-05;
  testPose.position.z = 0.301622770805;
  testPose.orientation.x = -5.11700663643e-07;
  testPose.orientation.y =  0.00431527188029;
  testPose.orientation.z = 0.000118577904216;
  testPose.orientation.w = 0.99999068214;
  PickPlace testArm;
  bool testSrvResp = testArm.setPose(testPose);
  EXPECT_EQ(typeid(testSrvResp), typeid(bool));
}

/**
 * @brief Test to check the member function setGripper of class
 * PickPlace
 */
TEST(TestPickPlace, testSetGripper) {
  PickPlace testArm;
  std::vector<double>open{0.01};
  bool testGripperState = testArm.setGripper(open);
  EXPECT_EQ(typeid(testGripperState), typeid(bool));
}

/**
 * @brief Test to check the member function executePick of class
 * PickPlace
 */
TEST(TestPickPlace, testExecutePick) {
  PickPlace manipulator;
  ros::NodeHandle t;
  bool testPick = manipulator.executePick(t);
  EXPECT_EQ(typeid(testPick), typeid(bool));
 }

 /**
  * @brief Test to check the member function executePlace of class
  * PickPlace
  */
 TEST(TestPickPlace, testExecutePlace) {
   PickPlace manipulator;
   ros::NodeHandle t;
   bool testPlace = manipulator.executePlace(t);
   EXPECT_EQ(typeid(testPlace), typeid(bool));
  }
