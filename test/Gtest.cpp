/**
 * @Author Gautam Balachandran
 * @file Gtest.cpp
 * @brief PID controller and Perception class testing
 *
 */

#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "../include/pidController.hpp"
#include "../include/turtlebotPerception.hpp"

/**
 * @brief Test case that checks if the calculated Euclidean Distance is correct
 */
TEST(PIDControllerTest, EuclideanDistancePass) {
  PidController pidController;
  tf::Point currentPosition, desiredPosition;
  currentPosition.x = 10;
  currentPosition.y = 10;
  currentPosition.z = 10;
  desiredPosition.x = 10;
  desiredPosition.y = 13;
  desiredPosition.z = 14;
  double expectPass = 5;
  double dist = pidController.euclideanDist(currentPosition, desiredPosition);
  EXPECT_NEAR(expectPass,dist,0.01);// Should Pass
}

/**
 * @brief Test case that checks if the calculated Euclidean Distance is of type double
 */
TEST(PIDControllerTest, EuclideanDistanceTypePass) {
  PidController pidController;
  tf::Point currentPosition, desiredPosition;
  currentPosition.x = 10;
  currentPosition.y = 10;
  currentPosition.z = 10;
  desiredPosition.x = 10;
  desiredPosition.y = 13;
  desiredPosition.z = 14;
  double expectPass = 5;
  double dist = pidController.euclideanDist(currentPosition, desiredPosition);
  EXPECT_EQ(typeid(dist), typeid(double));// Should Pass
}


/**
 * @brief Test case that checks if the calculated steering angle is correct
 */
TEST(PIDControllerTest, SteeringAnglePass) {
  PidController pidController;
  tf::Point trajPosition;
  double angVel = 30.0;
  trajPosition.x = 10;
  trajPosition.y = 10;
  trajPosition.z = 10;
  double expectPass = 30;
  double steeringAng = pidController.calcSteeringAng(trajPosition, angVel);
  EXPECT_NEAR(expectPass,steeringAng,0.01);// Should Pass
}

/**
 * @brief Test case that checks if the calculated Euclidean Distance is of type double
 */
TEST(PIDControllerTest, SteeringAngleTypePass) {
  PidController pidController;
  tf::Point trajPosition;
  double angVel = 30.0;
  trajPosition.x = 10;
  trajPosition.y = 10;
  trajPosition.z = 10;
  double expectPass = 0;
  double steeringAng = pidController.setSteeringAng(trajPosition, angVel);
  EXPECT_EQ(typeid(steeringAng), typeid(double));// Should Pass
}

/**
 * @brief Test case that checks if the type of PID Control parameters are correct
 */
TEST(PIDControllerTest, PidTypePass) {
  PidController pidController;
  tf::Point trajPosition, desiredPosition;
  double kP, kI, kD;
  trajPosition.x = 10;
  trajPosition.y = 10;
  trajPosition.z = 10;
  desiredPosition.x = 10;
  desiredPosition.y = 13;
  desiredPosition.z = 14;
  pidController.calcPID(currentPosition, desiredPosition);
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
TEST(TurtlebotPerceptionTest, DetecCollisionPass) {
  TurtlebotPerception turtlebotPerception;
  bool coll;
  coll = turtlebotPerception.detectCollision();
  EXPECT_TRUE(coll);// Should Pass
}

/**
 * @brief Test case that checks if the return value is boolean
 */
TEST(TurtlebotPerceptionTest, DetecCollisionTypePass) {
  TurtlebotPerception turtlebotPerception;
  bool coll;
  coll = turtlebotPerception.detectCollision();
  EXPECT_EQ(typeid(coll), typeid(bool));// Should Pass
}
