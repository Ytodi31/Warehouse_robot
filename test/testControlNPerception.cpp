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
#include <unistd.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
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
  pidController.setKP(10, 10);
  pidController.setKI(3.4, 4.5);
  pidController.setKD(4.5, 3);
  std::vector<double> kP, kD, kI;
  kP = pidController.getKP();
  kD = pidController.getKD();
  kI = pidController.getKI();
  EXPECT_EQ(typeid(kP), typeid(std::vector<double>));
  EXPECT_EQ(typeid(kD), typeid(std::vector<double>));
  EXPECT_EQ(typeid(kI), typeid(std::vector<double>));
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
  currentPose.getRotation().setEulerZYX(0, 0, 0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(0, 0, 0);
  double expectPass = 5;
  double dist = pidController.euclideanDist(currentPose, desiredPose);
  EXPECT_NEAR(expectPass, dist, 0.01);  // Should Pass
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
  currentPose.getRotation().setEulerZYX(0, 0, 0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(0, 0, 0);
  double dist = pidController.euclideanDist(currentPose, desiredPose);
  EXPECT_EQ(typeid(dist), typeid(double));  // Should Pass
}
/**
 * @brief Test case that checks if the calculated velocities is correct
 */
TEST(PIDControllerTest, VelocityPassNeg) {
  PidController pidController;
  ros::NodeHandle n;
  tf::Pose currentPose, desiredPose;
  double linearVel, angVel, expectPassLin = 0.8, expectPassAng = -0.1;
  currentPose.setOrigin(tf::Vector3(10, 10, 0));
  currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
  currentPose.setOrigin(tf::Vector3(14, 13, 0));
  desiredPose.setRotation(tf::createQuaternionFromRPY(0, 0, M_PI / 3));
  pidController.setKP(80, 80);
  pidController.setKD(3, 3);
  pidController.setKI(4, 4);
  pidController.setControllerNode(n);
  pidController.setVelocityPub();
  pidController.calcVel(currentPose, desiredPose);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_NEAR(expectPassLin, linearVel, 0.1);
  EXPECT_NEAR(expectPassAng, angVel, 0.1);
}

/**
 * @brief Test case that checks if the calculated velocities is correct
 * for an actual value of PID controller
 */
TEST(PIDControllerTest, VelocityPassPos) {
  PidController pidController;
  ros::NodeHandle n;
  tf::Pose currentPose, desiredPose;
  double linearVel, angVel, expectPassLin = 0.8, expectPassAng = -0.04;
  currentPose.setOrigin(tf::Vector3(10, 10, 0));
  currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
  currentPose.setOrigin(tf::Vector3(14, 13, 0));
  desiredPose.setRotation(tf::createQuaternionFromRPY(0, 0, M_PI / 3));
  pidController.setKP(0.02, 0.03);
  pidController.setKD(0.03, 0.05);
  pidController.setKI(0.04, 0.04);
  pidController.setControllerNode(n);
  pidController.setVelocityPub();
  pidController.calcVel(currentPose, desiredPose);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_NEAR(expectPassLin, linearVel, 0.1);
  EXPECT_NEAR(expectPassAng, angVel, 0.01);
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
  currentPose.getRotation().setEulerZYX(0, 0, 0);
  desiredPose.getOrigin().setX(14);
  desiredPose.getOrigin().setY(13);
  desiredPose.getOrigin().setZ(0);
  desiredPose.getRotation().setEulerZYX(0.1, 0, 0);
  pidController.setKP(80, 80);
  pidController.setKD(3, 3);
  pidController.setKI(4, 4);
  ros::NodeHandle n;
  pidController.setControllerNode(n);
  pidController.setVelocityPub();
  pidController.calcVel(currentPose, desiredPose);
  linearVel = pidController.getLinearVel();
  angVel = pidController.getAngularVel();
  EXPECT_EQ(typeid(linearVel), typeid(double));
  EXPECT_EQ(typeid(angVel), typeid(double));
}

/**
 * @brief Test case that checks if the marker is detected properly
 */
TEST(TurtlebotPerceptionTest, MarkerDetectionPass) {
  TurtlebotPerception turtlebotPerception;
  bool detectMarker;
  cv::Mat markerImage;
  std::string imagePath = ros::package::getPath("warehouse_robot");
  imagePath.append("/data/models/marker0/materials/textures/marker_test6.png");
  markerImage = cv::imread(imagePath);
  detectMarker = turtlebotPerception.detectArucoMarker(markerImage, 23);
  EXPECT_TRUE(detectMarker);  // Should Pass
}

/**
 * @brief Test case that checks if the marker is not detected for image with zeros
 */
TEST(TurtlebotPerceptionTest, MarkerDetectionPassNeg) {
  TurtlebotPerception turtlebotPerception;
  bool detectMarker;
  cv::Mat markerImage = cv::Mat::zeros(170, 170, CV_8UC1);
  detectMarker = turtlebotPerception.detectArucoMarker(markerImage, 23);
  EXPECT_FALSE(detectMarker);  // Should Pass
}
/**
 * @brief Test case that checks if the marker detector function returns boolean value
 */
TEST(TurtlebotPerceptionTest, MarkerDetectionTypePass) {
  TurtlebotPerception turtlebotPerception;
  bool detectMarker;
  cv::Mat markerImage;
  std::string imagePath = ros::package::getPath("warehouse_robot");
  imagePath.append("/data/models/marker0/materials/textures/marker_test6.png");
  markerImage = cv::imread(imagePath);
  detectMarker = turtlebotPerception.detectArucoMarker(markerImage, 23);
  EXPECT_EQ(typeid(detectMarker), typeid(bool));  // Should Pass
}

/**
 * @brief Test case that checks if the marker detector function returns boolean value
 */
TEST(TurtlebotPerceptionTest, calcVelPass) {
  TurtlebotPerception turtlebotPerception;
  bool detectMarker;
  cv::Mat markerImage;
  ros::NodeHandle n;
  turtlebotPerception.setControllerNode(n);
  turtlebotPerception.setPerceptionNode(n);
  turtlebotPerception.setKD(0.5);
  turtlebotPerception.setKP(0.1);
  turtlebotPerception.setKI(0.01);
  turtlebotPerception.setSubscribers();
  std::string imagePath = ros::package::getPath("warehouse_robot");
  imagePath.append("/data/models/marker0/materials/textures/marker_test6.png");
  markerImage = cv::imread(imagePath);
  detectMarker = turtlebotPerception.detectArucoMarker(markerImage, 23);
  geometry_msgs::Twist vel;
  vel = turtlebotPerception.calcVel();
  EXPECT_EQ(vel.linear.x, 0);
  EXPECT_EQ(vel.linear.y, 0);
  EXPECT_EQ(vel.linear.z, 0);
  EXPECT_EQ(vel.angular.x, 0);
  EXPECT_EQ(vel.angular.y, 0);
  EXPECT_EQ(vel.angular.z, 0);
}
