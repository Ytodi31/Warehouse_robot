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

 #include <../include/warehouse_robot/PickPlace.h>

 /**
  * @brief Test to check the member function checkGripperState of class
  * PickPlace
  */
TEST(TestPickPlace, testGripperState) {
  PickPlace manipulator;
  manipulator.robotState = 1;
  bool gripperState = manipulator.checkGripperState();
  // Checks if the gripper is closed when the robot is moving
  EXPECT_EQ(gripperState, true);
}

/**
 * @brief Test to check the member function setPick of class
 * PickPlace
 */
TEST(TestPickPlace, testSetPick) {
  PickPlace manipulator;
  geometry_msgs::Pose pickPose;
  pickPose.position.x = 1.0;
  pickPose.position.y = 2.0;
  pickPose.position.z = 3.0;
  pickPose.orientation.x = 4.0;
  pickPose.orientation.y = 5.0;
  pickPose.orientation.z = 6.0;
  pickPose.orientation.w = 7.0;
  manipulator.setPick();
  ASSERT_EQ(pickPose.position.x, manipulator.pickPose.position.x);
  ASSERT_EQ(pickPose.position.y, manipulator.pickPose.position.y);
  ASSERT_EQ(pickPose.position.z, manipulator.pickPose.position.z);
  ASSERT_EQ(pickPose.orientation.x, manipulator.pickPose.orientation.x);
  ASSERT_EQ(pickPose.orientation.y, manipulator.pickPose.orientation.y);
  ASSERT_EQ(pickPose.orientation.z, manipulator.pickPose.orientation.z);
  ASSERT_EQ(pickPose.orientation.w, manipulator.pickPose.orientation.w);
}

/**
 * @brief Test to check the member function setPlace of class
 * PickPlace
 */
TEST(TestPickPlace, testSetPlace) {
  PickPlace manipulator;
  geometry_msgs::Pose placePose;
  placePose.position.x = 8;
  placePose.position.y = 9;
  placePose.position.z = 10;
  placePose.orientation.x = 11;
  placePose.orientation.y = 12;
  placePose.orientation.z = 13;
  placePose.orientation.w = 14;
  manipulator.setPick();
  ASSERT_EQ(placePose.position.x, manipulator.placePose.position.x);
  ASSERT_EQ(placePose.position.y, manipulator.placePose.position.y);
  ASSERT_EQ(placePose.position.z, manipulator.placePose.position.z);
  ASSERT_EQ(placePose.orientation.x, manipulator.placePose.orientation.x);
  ASSERT_EQ(placePose.orientation.y, manipulator.placePose.orientation.y);
  ASSERT_EQ(placePose.orientation.z, manipulator.placePose.orientation.z);
  ASSERT_EQ(placePose.orientation.w, manipulator.placePose.orientation.w);
}
