/*
 * pidController.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: gautam
 */

#include <pidController.h>

/**
 * @brief Mock of the method to calculate the controller parameters
 * @param Trajectory position and current position
 * @return none
 */
void pidController::calcPID(tf::Point trajPosition, tf::Point currentPosition) {
  double kP, kD, kI;
  kP = 0.3;
  kD = 0.05;
  kI = 1.02;  //  MOCK VALUES FOR TESTING!
  pidController.setKP(kP);
  pidController.setKD(kD);
  pidController.setKI(kI);
}

/**
 * @brief Mock of the euclidean distance calculator
 * @param Current position and desired position
 * @return calculated distance
 */
double pidController::euclideanDist(tf::Point currentPosition,
                                    tf::Point desiredPosition) {
  double dist = 5;  // MOCK VALUE FOR TESTING!
  return dist;
}

/**
 * @brief Mock of the method to calculate the steering angle for the robot
 * @param Trajectory position and the angular velocity
 * @return calculated steering angle
 */
double pidController::calcSteeringAng(tf::Point trajPosition,
                                      double angVelocity) {
  double steeringAng = 30;  // MOCK VALUE FOR TESTING!
  return steeringAng;
}
