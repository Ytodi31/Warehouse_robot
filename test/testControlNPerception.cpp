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
 * @file testControlNPerception.cpp
 * @brief This file tests the member functions of class pidController and
 * turtlebotPerception
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

#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "../include/warehouse_robot/pidController.hpp"
#include "../include/warehouse_robot/turtlebotPerception.hpp"

/**
 * @brief Test case that checks if the calculated Euclidean Distance is correct
 */
TEST(PIDControllerTest, EuclideanDistancePass) {
  PidController pidController;
  tf::Point currentPosition, desiredPosition;
  currentPosition.setX(10);
  currentPosition.setY(10);
  currentPosition.setZ(10);
  desiredPosition.setX(10);
  desiredPosition.setY(13);
  desiredPosition.setZ(14);
  double expectPass = 5;
  double dist = pidController.euclideanDist(currentPosition, desiredPosition);
  EXPECT_NEAR(expectPass,dist,0.01);  // Should Pass
}

/**
 * @brief Test case that checks if the calculated Euclidean Distance is of type
 * double
 */
TEST(PIDControllerTest, EuclideanDistanceTypePass) {
  PidController pidController;
  tf::Point currentPosition, desiredPosition;
  currentPosition.setX(10);
  currentPosition.setY(10);
  currentPosition.setZ(10);
  desiredPosition.setX(10);
  desiredPosition.setY(13);
  desiredPosition.setZ(14);
  double dist = pidController.euclideanDist(currentPosition, desiredPosition);
  EXPECT_EQ(typeid(dist), typeid(double));  // Should Pass
}

/**
 * @brief Test case that checks if the calculated velocities is correct
 */
TEST(PIDControllerTest, VelocityPass) {
  PidController pidController;
  tf::Point currentPosition, desiredPosition;
  double linearVel, angVel, expectPassLin = 10, expectPassAng = 10;
  currentPosition.setX(10);
  currentPosition.setY(10);
  currentPosition.setZ(10);
  desiredPosition.setX(10);
  desiredPosition.setY(13);
  desiredPosition.setZ(14);
  pidController.calcVel(currentPosition, desiredPosition);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_NEAR(expectPassLin,linearVel,0.01);  // Should Pass
  EXPECT_NEAR(expectPassAng,angVel,0.01);// Should Pass
}

/**
 * @brief Test case that checks if the calculated velocities are of type
 * double
 */
TEST(PIDControllerTest, VelocityTypePass) {
  PidController pidController;
  tf::Point currentPosition, desiredPosition;
  double linearVel, angVel, expectPassLin = 10, expectPassAng = 10;
  currentPosition.setX(10);
  currentPosition.setY(10);
  currentPosition.setZ(10);
  desiredPosition.setX(10);
  desiredPosition.setY(13);
  desiredPosition.setZ(14);
  pidController.calcVel(currentPosition, desiredPosition);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_EQ(typeid(linearVel), typeid(double));// Should Pass
  EXPECT_EQ(typeid(angVel), typeid(double));// Should Pass
}

/**
 * @brief Test case that checks if the type of PID Control parameters are
 * correct
 */
TEST(PIDControllerTest, PidTypePass) {
  PidController pidController;
  tf::Point trajPosition, desiredPosition;
  double kP, kI, kD;
  kP = pidController.getKP();
  kD = pidController.getKD();
  kI = pidController.getKI();
  EXPECT_EQ(typeid(kP), typeid(double));// Should Pass
  EXPECT_EQ(typeid(kD), typeid(double));// Should Pass
  EXPECT_EQ(typeid(kI), typeid(double));// Should Pass
}

/**
 * @brief Test case that checks if the collision is detected
 */
TEST(TurtlebotPerceptionTest, DetectCollisionPass) {
  TurtlebotPerception turtlebotPerception;
  bool coll;
  coll = turtlebotPerception.detectCollision();
  EXPECT_TRUE(coll);  // Should Pass
}

/**
 * @brief Test case that checks if the return value is boolean
 */
TEST(TurtlebotPerceptionTest, DetectCollisionTypePass) {
  TurtlebotPerception turtlebotPerception;
  bool coll;
  coll = turtlebotPerception.detectCollision();
  EXPECT_EQ(typeid(coll), typeid(bool));  // Should Pass
}
