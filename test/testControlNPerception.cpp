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
#include <opencv2/aruco.hpp>
#include "../include/warehouse_robot/pidController.hpp"
#include "../include/warehouse_robot/turtlebotPerception.hpp"

/**
 * @brief Test case that checks the pose getter method
 */
TEST(PIDControllerTest, PoseTypeTest) {
  PidController pidController;
  tf::Pose pose;
  pose = pidController.getPose();
  EXPECT_EQ(typeid(pose), typeid(tf::Pose));  // Should Pass
}
/**
 * @brief Test case that checks the linear velocity getter method
 */
TEST(PIDControllerTest, LinearVelTypeTest) {
  PidController pidController;
  double vel;
  vel = pidController.getLinearVel();
  EXPECT_EQ(typeid(vel), typeid(double));  // Should Pass
}
/**
 * @brief Test case that checks the angular velocity getter method
 */
TEST(PIDControllerTest, AngularVelTypeTest) {
  PidController pidController;
  double vel;
  vel = pidController.getAngularVel();
  EXPECT_EQ(typeid(vel), typeid(double));  // Should Pass
}
/**
 * @brief Test case that checks the control parameters getter methods
 */
TEST(PIDControllerTest, ControlParametersTypeTest) {
  PidController pidController;
  double kP,kD,kI;
  kP = pidController.getKP();
  kD = pidController.getKD();
  kI = pidController.getKI();
  EXPECT_EQ(typeid(kP), typeid(double));  // Should Pass
  EXPECT_EQ(typeid(kD), typeid(double));// Should Pass
  EXPECT_EQ(typeid(kI), typeid(double));// Should Pass
}
/**
 * @brief Test case that checks if the calculated Euclidean Distance is correct
 */
TEST(PIDControllerTest, EuclideanDistancePass) {
  PidController pidController;
  tf::Pose currentPose, desiredPose;
  currentPose.getOrigin().setX(10);
  currentPose.getOrigin().setY(10);
  currentPose.getOrigin().setZ(0);
  currentPose.getRotation().setEulerZYX(0,0,0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(0,0,0);
  double expectPass = 5;
  double dist = pidController.euclideanDist(currentPose, desiredPose);
  EXPECT_NEAR(expectPass,dist,0.01);  // Should Pass
}
/**
 * @brief Test case that checks if the calculated Euclidean Distance is of 
 * type double
 */
TEST(PIDControllerTest, EuclideanDistanceTypePass) {
  PidController pidController;
  tf::Pose currentPose, desiredPose;
  currentPose.getOrigin().setX(10);
  currentPose.getOrigin().setY(10);
  currentPose.getOrigin().setZ(0);
  currentPose.getRotation().setEulerZYX(0,0,0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(0,0,0);
  double dist = pidController.euclideanDist(currentPose, desiredPose);
  EXPECT_EQ(typeid(dist), typeid(double));  // Should Pass
}
/**
 * @brief Test case that checks if the calculated velocities is correct
 */
TEST(PIDControllerTest, VelocityPass) {
  PidController pidController;
  tf::Pose currentPose, desiredPose;
  double linearVel, angVel, expectPassLin = 0.3, expectPassAng = 5;
  currentPose.getOrigin().setX(10);
  currentPose.getOrigin().setY(10);
  currentPose.getRotation().setEulerZYX(0,0,0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(30,0,0);
  pidController.setKP(80);
  pidController.setKD(3);
  pidController.setKI(4);
  pidController.calcVel(currentPose, desiredPose);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_NEAR(expectPassLin,linearVel,0.1);  // Should Pass
  EXPECT_NEAR(expectPassAng,angVel,0.1);// Should Pass
}
/**
 * @brief Test case that checks if the calculated velocities are of type
 * double
 */
TEST(PIDControllerTest, VelocityTypePass) {
  PidController pidController;
  tf::Pose currentPose, desiredPose;
  double linearVel, angVel;
  currentPose.getOrigin().setX(10);
  currentPose.getOrigin().setY(10);
  currentPose.getOrigin().setZ(0);
  currentPose.getRotation().setEulerZYX(0,0,0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(30,0,0);
  pidController.setKP(80);
  pidController.setKD(3);
  pidController.setKI(4);
  pidController.calcVel(currentPose, desiredPose);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_EQ(typeid(linearVel), typeid(double));  // Should Pass
  EXPECT_EQ(typeid(angVel), typeid(double));// Should Pass
}
/**
 * @brief Test case that checks if the getter method of the collision variable
 */
TEST(TurtlebotPerceptionTest, CollisionTypePass) {
  TurtlebotPerception turtlebotPerception;
  bool coll;
  coll = turtlebotPerception.getCollide();
  EXPECT_EQ(typeid(coll), typeid(bool));  // Should Pass
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
 * @brief Test case that checks if the collision detector return value is 
 * boolean
 */
TEST(TurtlebotPerceptionTest, DetectCollisionTypePass) {
  TurtlebotPerception turtlebotPerception;
  bool coll;
  coll = turtlebotPerception.detectCollision();
  EXPECT_EQ(typeid(coll), typeid(bool));  // Should Pass
}
/**
 * @brief Test case that checks if the marker is detected properly
 */
TEST(TurtlebotPerceptionTest, MarkerDetectionPass) {
  TurtlebotPerception turtlebotPerception;
  bool detectMarker;
  cv::Mat markerImage;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::drawMarker(dictionary, 23, 170, markerImage, 1);
  detectMarker = turtlebotPerception.detectArucoMarker(markerImage, 23);
  EXPECT_TRUE(detectMarker);  // Should Pass
}
/**
 * @brief Test case that checks if the marker detector function returns boolean value
 */
TEST(TurtlebotPerceptionTest, MarkerDetectionTypePass) {
  TurtlebotPerception turtlebotPerception;
  bool detectMarker;
  cv::Mat markerImage;
  cv::Ptr<cv:aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::drawMarker(dictionary, 23, 170, markerImage, 1);
  detectMarker = turtlebotPerception.detectArucoMarker(markerImage, 23);
  EXPECT_EQ(typeid(detectMarker), typeid(bool));  // Should Pass
}

